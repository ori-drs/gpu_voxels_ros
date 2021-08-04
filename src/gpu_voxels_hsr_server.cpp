//----------------------------------------------------------------------
/*!\file
 *
 * \author  Mark Finean
 * \date    2020-09-24 
 * \author  Christian Jülg
 * \date    2015-08-07
 * \author  Andreas Hermann
 * \date    2016-12-24
 */
//----------------------------------------------------------------------
#include <gpu_voxels_ros/gpu_voxels_hsr_server.h>
#include <gpu_voxels_ros/timing.h>

namespace gpu_voxels_ros{

  GPUVoxelsHSRServer::GPUVoxelsHSRServer(ros::NodeHandle& node) {
    node_ = node;

    T_B_C_ = Matrix4f(1, 0, 0, 0,
                      0, 1, 0, 0,
                      0, 0, 1, 0,
                      0, 0, 0, 1);

    T_D_B_ = Matrix4f(1, 0, 0, 0,
                      0, 1, 0, 0,
                      0, 0, 1, 0,
                      0, 0, 0, 1);

    remove_floor_ = true;


    node_.param<std::string>("transform_topic", transform_topic_, "/kinect/vrpn_client/estimated_transform");
    // node_.param<std::string>("pcl_topic", pcl_topic_, "/camera/depth_registered/points");
    node_.param<std::string>("pcl_topic", pcl_topic_, "/hsrb/head_rgbd_sensor/depth_registered/rectified_points");
    node_.param<std::string>("sensor_frame", sensor_frame_, "head_rgbd_sensor_rgb_frame");


    // node_.param<float>("voxel_side_length", voxel_side_length_, 0.1f);
    // node_.param<int>("map_size_x", (int&) map_dimensions_.x, 256);
    // node_.param<int>("map_size_y", (int&) map_dimensions_.y, 256);
    // node_.param<int>("map_size_z", (int&) map_dimensions_.z, 32);


    node_.param<float>("voxel_side_length", voxel_side_length_, 0.05f);
    node_.param<int>("map_size_x", (int&) map_dimensions_.x, 256);
    node_.param<int>("map_size_y", (int&) map_dimensions_.y, 256);
    node_.param<int>("map_size_z", (int&) map_dimensions_.z, 64);

    // Iros params
    // node_.param<float>("voxel_side_length", voxel_side_length_, 0.025f);
    // node_.param<int>("map_size_x", (int&) map_dimensions_.x, 448);
    // node_.param<int>("map_size_y", (int&) map_dimensions_.y, 448);
    // node_.param<int>("map_size_z", (int&) map_dimensions_.z, 128);

    // node_.param<int>("map_size_x", (int&) map_dimensions_.x, 704);
    // node_.param<int>("map_size_y", (int&) map_dimensions_.y, 704);
    // node_.param<int>("map_size_z", (int&) map_dimensions_.z, 128);
    
    // node_.param<float>("voxel_side_length", voxel_side_length_, 0.02f);
    // node_.param<int>("map_size_x", (int&) map_dimensions_.x, 512);
    // node_.param<int>("map_size_y", (int&) map_dimensions_.y, 512);
    // node_.param<int>("map_size_z", (int&) map_dimensions_.z, 128);

    map_pub_ = node.advertise<pcl::PointCloud<pcl::PointXYZI> >("gpu_voxels_pointcloud", 1, true);
    ground_sdf_pub_ = node.advertise<pcl::PointCloud<pcl::PointXYZI> >("gpu_voxels_ground_sdf", 1, true);
    update_time_pub_ = node.advertise<pcl::PointCloud<pcl::PointXYZI> >("gpu_voxels_update_times", 1, true);
    ground_sdf_grad_pub_ = node.advertise<visualization_msgs::MarkerArray>("gpu_voxels_ground_sdf_grad", 2000, true);
    cone_flag_pub_ = node.advertise<pcl::PointCloud<pcl::PointXYZI> >("gpu_voxels_cone_flags", 1, true);
    traj_sweep_pub_ = node.advertise<pcl::PointCloud<pcl::PointXYZI> >("gpu_voxels_traj_steps", 1, true);
    cone_arrow_pub_ = node.advertise<visualization_msgs::MarkerArray>( "cone_arrows", 1, true);

    costmap_pub_ = node.advertise<pcl::PointCloud<pcl::PointXYZI> >("gpu_voxels_costmap", 1, true);


    transform_sub_ = node.subscribe(transform_topic_, 10, &GPUVoxelsHSRServer::PoseCallback, this);
    pcl_sub_ = node.subscribe(pcl_topic_, 10, &GPUVoxelsHSRServer::PointcloudCallback, this);


    // Generate a GPU-Voxels instance:
    gvl_ = gpu_voxels::GpuVoxels::getInstance();
    gvl_->initialize(map_dimensions_.x, map_dimensions_.y, map_dimensions_.z, voxel_side_length_);
    gvl_->addMap(MT_SIGNED_DISTANCE_VOXELMAP, "pbaDistanceVoxmap");
    gvl_->addMap(MT_PROBAB_VOXELMAP, "maintainedProbVoxmap");
    gvl_->addMap(MT_DISTANCE_VOXELMAP, "pbaDistanceVoxmapVisual");
    gvl_->addMap(MT_PROBAB_VOXELMAP, "myRobotMap");
    gvl_->addMap(MT_PROBAB_VOXELMAP, "cleanVoxmap");
    gvl_->addMap(MT_PROBAB_VOXELMAP, "cleanVoxmapVisual");

    // gvl_->addMap(MT_BITVECTOR_VOXELMAP, "myBitRobotMap");

    // Add the robot model
    gvl_->addRobot("hsrRobot", "hsr/originals/mod_hsrb.urdf", true);
    std::cout << "Added HSR Urdf model" << std::endl;

    robot_ptr_ = dynamic_pointer_cast<gpu_voxels::robot::UrdfRobot>(gvl_->getRobot("hsrRobot"));

    robotVoxmap_ = dynamic_pointer_cast<ProbVoxelMap>(gvl_->getMap("myRobotMap"));

    pbaDistanceVoxmap_ = dynamic_pointer_cast<DistanceVoxelMap>(gvl_->getMap("pbaDistanceVoxmap"));
    pbaInverseDistanceVoxmap_ = dynamic_pointer_cast<DistanceVoxelMap>(gvl_->getMap("pbaDistanceVoxmapInverse"));
    maintainedProbVoxmap_ = dynamic_pointer_cast<ProbVoxelMap>(gvl_->getMap("maintainedProbVoxmap"));
    pbaDistanceVoxmapVisual_ = dynamic_pointer_cast<DistanceVoxelMap>(gvl_->getMap("pbaDistanceVoxmapVisual"));
    signedDistanceMap_ = dynamic_pointer_cast<InheritSignedDistanceVoxelMap>(pbaDistanceVoxmap_);
    // cleanVoxmap_ = dynamic_pointer_cast<ProbVoxelMap>(gvl_->getMap("cleanVoxmap"));
    // cleanVoxmapVisual_ = dynamic_pointer_cast<ProbVoxelMap>(gvl_->getMap("cleanVoxmapVisual"));

    // sdf_grad_map_ = std::vector<gpu_voxels::VectorSdfGrad>(pbaDistanceVoxmap_->getVoxelMapSize());
    sdf_map_ = std::vector<float>(pbaDistanceVoxmap_->getVoxelMapSize());
    // time_update_map_ = std::vector<uint>(maintainedProbVoxmap_->getVoxelMapSize());
    
    // occupancy_map_ = std::vector<int>(pbaDistanceVoxmap_->getVoxelMapSize());
    // flag_map_ = std::vector<bool>(maintainedProbVoxmap_->getVoxelMapSize(), false);

    // traj_step_map_ = std::vector<int>(maintainedProbVoxmap_->getVoxelMapSize());

    host_costmap_ = std::vector<float>(pbaDistanceVoxmap_->getVoxelMapSize());


    maintainedProbVoxmap_->clearMap();
    maintainedProbVoxmap_->resetTimeSteps();
    // maintainedProbVoxmap_->getVoxelUpdateTimesToHost(time_update_map_);
    // float test = getPercentageMapExplored();

    std::cout << "HSR Server Ready" << std::endl;
  }

  void GPUVoxelsHSRServer::CallbackSync(){

    timing::Timer sync_callback_timer("CallbackSync");

    ros::Time pcl_time;
    double time_delay = 2e-3;
    timing::Timer time_increment_timer("time_increment_timer");
    maintainedProbVoxmap_->incrementTimeSteps();
    time_increment_timer.Stop();

    while (!pointcloud_queue_.empty()) {
        
        timing::Timer update_transform_timer("TransformUpdate");
        
        // First loop until we get a camera pose transform within our tolderance
        bool new_pos = false;
        pcl_time = pointcloud_queue_.front()->header.stamp;
        while (cam_transform_queue_.size() > 1 &&
            std::get<0>(cam_transform_queue_.front()) <= pcl_time + ros::Duration(time_delay)) {
              sync_tf_ = std::get<1>(cam_transform_queue_.front());
              cam_transform_queue_.pop();
              new_pos = true;
        }
        if (cam_transform_queue_.empty()
            || std::get<0>(cam_transform_queue_.front()) <= pcl_time + ros::Duration(time_delay)) {
              break;
        }
        if (!new_pos) {
              pointcloud_queue_.pop();
              continue;
        }

        sensor_msgs::PointCloud2::ConstPtr msg = pointcloud_queue_.front();
        pcl::fromROSMsg(*msg, cloud_);
        
        if ((int) cloud_.size()==0) {
              pointcloud_queue_.pop();
              continue;
        }

        Vector3f camera_pos = Vector3f(sync_tf_.a14, sync_tf_.a24, sync_tf_.a34)/sync_tf_.a44;

        // std::cout << camera_pos << std::endl;
        update_transform_timer.Stop();

        timing::Timer transform_pc_timer("ResizePC");
        
        std::vector<Vector3f> point_data;

        // Reset/clear the pointcloud to insert
        point_data.resize(cloud_.points.size());

        for (uint32_t i = 0; i < cloud_.points.size(); i++)
        {
          point_data[i].x = cloud_.points[i].x;
          point_data[i].y = cloud_.points[i].y;
          point_data[i].z = cloud_.points[i].z;
        }

        my_point_cloud_.update(point_data);
        
        // transform new pointcloud to world coordinates
        my_point_cloud_.transformSelf(&sync_tf_);
        
        transform_pc_timer.Stop();

        timing::Timer update_esdf_timer("UpdateESDF");

        pbaDistanceVoxmap_->clearMap();
        // cleanVoxmap_->clearMap();

        maintainedProbVoxmap_->insertClippedSensorData<BIT_VECTOR_LENGTH>(my_point_cloud_, camera_pos, true, false, eBVM_OCCUPIED, 
                                                                          min_ray_length_, max_ray_length_, NULL, remove_floor_);

        
        // maintainedProbVoxmap_->erodeInto(*cleanVoxmap_, 2); //erode only "lonely voxels" without occupied neighbors
        // maintainedProbVoxmap_->erodeLonelyInto(*cleanVoxmap_); //erode only "lonely voxels" without occupied neighbors

        // signedDistanceMap_->occupancyMerge(cleanVoxmap_, 0.75, 0.74999);
        signedDistanceMap_->occupancyMerge(maintainedProbVoxmap_, 0.75, 0.74999);

        signedDistanceMap_->parallelBanding3DUnsigned();

        update_esdf_timer.Stop();

        timing::Timer transfer_timer("HostRetrieval");
        // maintainedProbVoxmap_->getVoxelUpdateTimesToHost(time_update_map_);
        
        pbaDistanceVoxmap_->getUnsignedDistancesToHost(sdf_map_);


        transfer_timer.Stop();
        sync_callback_timer.Stop();
        pointcloud_queue_.pop();

        
        // publishRVIZOccupancy(sdf_grad_map_);
        // publishRVIZGroundSDF(sdf_grad_map_);

        // maintainedProbVoxmap_->getVoxelFlagsToHost(flag_map_);
        // publishRVIZVoxelFlags(flag_map_);
       
        // pbaDistanceVoxmapVisual_->clone(*(pbaDistanceVoxmap_.get()));
        // cleanVoxmapVisual_->clone(*(cleanVoxmap_.get()));
        // gvl_->visualizeMap("myRobotMap");
        // gvl_->visualizeMap("pbaDistanceVoxmapVisual");
        // gvl_->visualizeMap("myBitRobotMap");
        // gvl_->visualizeMap("cleanVoxmapVisual");

        // publishRVIZTrajSweepOccupancy(traj_step_map_);
        publishRVIZCostmap(host_costmap_);
        publishRVIZConeRankings();

        // publishRVIZUpdateTimes(time_update_map_, 5);

        publishRVIZGroundSDF(sdf_map_);
        publishRVIZOccupancy(sdf_map_);


        // publishRVIZOccupancy(occupancy_map_);
        // std::cout << "Finished publishing" << std::endl;
        // timing::Timing::Print(std::cout);

    }
    // publishRVIZGroundSDFGrad(sdf_grad_map_);
  }

  float GPUVoxelsHSRServer::getPercentageMapExplored() const{
    // uint max_val = 0;

    // for (size_t x = 0; x < map_dimensions_.x; ++x){
    //   for (size_t y = 0; y < map_dimensions_.y; ++y){
    //     for (size_t z = 0; z < map_dimensions_.z; ++z) {
    //       size_t ind = z * map_dimensions_.x * map_dimensions_.y + y * map_dimensions_.x + x;
    //       max_val = std::max(max_val, (uint) time_update_map_[ind]);
    //       // std::cout << time_update_map_[ind] << std::endl;
    //     }
    //   }
    // }
    // std::cout << "Max val: " << max_val << std::endl; 
    uint voxels_observed = maintainedProbVoxmap_->getMapCoverage();
    std::cout << "Number of voxels observed: " << voxels_observed << " out of: " << maintainedProbVoxmap_->getVoxelMapSize() << std::endl;
    return (float) voxels_observed / (float) maintainedProbVoxmap_->getVoxelMapSize();
  }

  void GPUVoxelsHSRServer::publishRVIZVoxelFlags(const std::vector<bool> &flag_map) {
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // int ctr = 0;
    // int ctr_else = 0;

    for (size_t x = 0; x < map_dimensions_.x; ++x)
      for (size_t y = 0; y < map_dimensions_.y; ++y)
        for (size_t z = 0; z < map_dimensions_.z; ++z) {
          size_t ind = z * map_dimensions_.x * map_dimensions_.y + y * map_dimensions_.x + x;
          if (flag_map[ind]) {
            pt.x = ((float) x - (0.5 * (float) map_dimensions_.x)) * voxel_side_length_;
            pt.y = ((float)y - (0.5 * (float) map_dimensions_.y)) * voxel_side_length_;
            pt.z = (float)z * (float) voxel_side_length_;
            cloud.push_back(pt);
            // ctr+=1;
          }
          else{
            // ctr_else+=1;
            continue;
          }
    }

    // std::cout << "Numer of true voxel flags: " << ctr << std::endl;
    // std::cout << "Numer of false voxel flags: " << ctr_else << std::endl;

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "odom";
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    cone_flag_pub_.publish(cloud_msg);
  }

  void GPUVoxelsHSRServer::publishRVIZCostmap(const std::vector<float> &costmap) {
    pcl::PointXYZI pt;
    pcl::PointCloud<pcl::PointXYZI> cloud;

    for (size_t x = 0; x < map_dimensions_.x; ++x)
      for (size_t y = 0; y < map_dimensions_.y; ++y)
        for (size_t z = 0; z < map_dimensions_.z; ++z) {
          size_t ind = z * map_dimensions_.x * map_dimensions_.y + y * map_dimensions_.x + x;
          // Priority 1
          if (costmap[ind] > 1e5) {
            pt.x = ((float) x - (0.5 * (float) map_dimensions_.x)) * voxel_side_length_;
            pt.y = ((float)y - (0.5 * (float) map_dimensions_.y)) * voxel_side_length_;
            pt.z = (float)z * (float) voxel_side_length_;
            pt.intensity = 1;
            cloud.push_back(pt);
          }
          if (costmap[ind] > 1 && costmap[ind] < 1e5){
            pt.x = ((float) x - (0.5 * (float) map_dimensions_.x)) * voxel_side_length_;
            pt.y = ((float)y - (0.5 * (float) map_dimensions_.y)) * voxel_side_length_;
            pt.z = (float)z * (float) voxel_side_length_;
            pt.intensity = 0.5;
            cloud.push_back(pt);
          }         
          else{
            continue;
          }
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "odom";
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    costmap_pub_.publish(cloud_msg);
  }

  void GPUVoxelsHSRServer::publishRVIZUpdateTimes(const std::vector<uint16_t> &time_map, uint16_t threshold) {
    pcl::PointXYZI pt;
    pcl::PointCloud<pcl::PointXYZI> cloud;

    for (size_t x = 0; x < map_dimensions_.x; ++x)
      for (size_t y = 0; y < map_dimensions_.y; ++y)
        for (size_t z = 0; z < map_dimensions_.z; ++z) {
          size_t ind = z * map_dimensions_.x * map_dimensions_.y + y * map_dimensions_.x + x;
          if (time_map[ind] <= threshold) {
            pt.x = ((float) x - (0.5 * (float) map_dimensions_.x)) * voxel_side_length_;
            pt.y = ((float)y - (0.5 * (float) map_dimensions_.y)) * voxel_side_length_;
            pt.z = (float)z * (float) voxel_side_length_;
            pt.intensity = time_map[ind];
            cloud.push_back(pt);
          }
          else{
            continue;
          }
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "odom";
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    update_time_pub_.publish(cloud_msg);
  }

  void GPUVoxelsHSRServer::publishRVIZTrajSweepOccupancy(const std::vector<int> &occupancy_map) {
    pcl::PointXYZI pt;
    pcl::PointCloud<pcl::PointXYZI> cloud;
 
    // int min_val = 100000;

    for (size_t x = 0; x < map_dimensions_.x; ++x){
      for (size_t y = 0; y < map_dimensions_.y; ++y){
        for (size_t z = 0; z < map_dimensions_.z; ++z) {
          size_t ind = z * map_dimensions_.x * map_dimensions_.y + y * map_dimensions_.x + x;
          int traj_step = occupancy_map[ind];
          // min_val = std::min(min_val, traj_step);
          if (traj_step != (UNKNOWN_PROBABILITY + (int) eBVM_UNCERTAIN_OCC_PROB - (int) eBVM_SWEPT_VOLUME_START + 1) ) {
            pt.x = ((float) x - 0.5 * (float) map_dimensions_.x) * voxel_side_length_;
            pt.y = ((float)y - 0.5 * (float) map_dimensions_.y) * voxel_side_length_;
            pt.z = (float)z * (float) voxel_side_length_;
            pt.intensity = traj_step;

            cloud.push_back(pt);
          } //if
          else{
            continue;
          } //else

        } // for z
      } //for y
    } // for x
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "odom";
    sensor_msgs::PointCloud2 cloud_msg;

    // std::cout << "min_traj_step =" << min_val << std::endl;;
    pcl::toROSMsg(cloud, cloud_msg);
    traj_sweep_pub_.publish(cloud_msg);
  }

  void GPUVoxelsHSRServer::publishRVIZOccupancy(const std::vector<int> &occupancy_map) {
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    for (size_t x = 0; x < map_dimensions_.x; ++x)
      for (size_t y = 0; y < map_dimensions_.y; ++y)
        for (size_t z = 0; z < map_dimensions_.z; ++z) {
          size_t ind = z * map_dimensions_.x * map_dimensions_.y + y * map_dimensions_.x + x;
          if (occupancy_map[ind] == 1) {
            pt.x = ((float) x - 0.5 * (float) map_dimensions_.x) * voxel_side_length_;
            pt.y = ((float)y - 0.5 * (float) map_dimensions_.y) * voxel_side_length_;
            pt.z = (float)z  * (float) voxel_side_length_;
            cloud.push_back(pt);
          }
          else{
            continue;
          }
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "odom";
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    map_pub_.publish(cloud_msg);
  }

  void GPUVoxelsHSRServer::publishRVIZOccupancy(const std::vector<float> &sdf_map) {
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    for (size_t x = 0; x < map_dimensions_.x; ++x)
      for (size_t y = 0; y < map_dimensions_.y; ++y)
        for (size_t z = 0; z < map_dimensions_.z; ++z) {
          size_t ind = z * map_dimensions_.x * map_dimensions_.y + y * map_dimensions_.x + x;
          if (sdf_map[ind] <= 0) {
            pt.x = ((float) x - (0.5 * (float) map_dimensions_.x)) * voxel_side_length_;
            pt.y = ((float)y - (0.5 * (float) map_dimensions_.y)) * voxel_side_length_;
            pt.z = (float)z * (float) voxel_side_length_;
            cloud.push_back(pt);
          }
          else{
            continue;
          }
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "odom";
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    map_pub_.publish(cloud_msg);
  }

  void GPUVoxelsHSRServer::publishRVIZOccupancy(const std::vector<gpu_voxels::VectorSdfGrad> &sdf_grad_map) {
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    for (size_t x = 0; x < map_dimensions_.x; ++x)
      for (size_t y = 0; y < map_dimensions_.y; ++y)
        for (size_t z = 0; z < map_dimensions_.z; ++z) {
          size_t ind = z * map_dimensions_.x * map_dimensions_.y + y * map_dimensions_.x + x;
          if (sdf_grad_map[ind].sdf <= 0) {
            pt.x = ((float) x - (0.5 * (float) map_dimensions_.x)) * voxel_side_length_;
            pt.y = ((float)y - (0.5 * (float) map_dimensions_.y)) * voxel_side_length_;
            pt.z = (float)z * (float) voxel_side_length_;
            cloud.push_back(pt);
          }
          else{
            continue;
          }
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "odom";
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    map_pub_.publish(cloud_msg);
  }

  void GPUVoxelsHSRServer::publishRVIZGroundSDF(const std::vector<gpu_voxels::VectorSdfGrad> &sdf_grad_map) {
  
    pcl::PointXYZI pt;
    pcl::PointCloud<pcl::PointXYZI> cloud;

    for (size_t x = 0; x < map_dimensions_.x; ++x){
      for (size_t y = 0; y < map_dimensions_.y; ++y){
          size_t ind = y * map_dimensions_.x + x;

          if (sdf_grad_map[ind].sdf <= 0.5)
          {            
            pt.x = ((float) x - (0.5 * (float) map_dimensions_.x)) * voxel_side_length_;
            pt.y = ((float)y - (0.5 * (float) map_dimensions_.y)) * voxel_side_length_;
            pt.z = 0;
            float dist = 1000;
            for (size_t z = 0; z < map_dimensions_.z; ++z){
              dist = std::min(dist, sdf_grad_map_[ind].sdf);
            }
            pt.intensity = dist;
            cloud.push_back(pt);
          }

          if (sdf_grad_map[ind].sdf >0.5)
          {            
            pt.x = ((float) x - (0.5 * (float) map_dimensions_.x)) * voxel_side_length_;
            pt.y = ((float)y - (0.5 * (float) map_dimensions_.y)) * voxel_side_length_;
            pt.z = 0;
            pt.intensity = 0.5;
            cloud.push_back(pt);
          }
        }
    }
    
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "odom";
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    ground_sdf_pub_.publish(cloud_msg);

  }

  void GPUVoxelsHSRServer::publishRVIZGroundSDF(const std::vector<float> &sdf_map) {
  
    pcl::PointXYZI pt;
    pcl::PointCloud<pcl::PointXYZI> cloud;

    for (size_t x = 0; x < map_dimensions_.x; ++x){
      for (size_t y = 0; y < map_dimensions_.y; ++y){
          size_t ind = y * map_dimensions_.x + x;

          if (sdf_map[ind] <= 0.5)
          {            
            pt.x = ((float) x - (0.5 * (float) map_dimensions_.x)) * voxel_side_length_;
            pt.y = ((float)y - (0.5 * (float) map_dimensions_.y)) * voxel_side_length_;
            pt.z = 0;
            float dist = 1000;
            for (size_t z = 0; z < map_dimensions_.z; ++z){
              dist = std::min(dist, sdf_map[ind]);
            }
            pt.intensity = dist;
            cloud.push_back(pt);
          }

          if (sdf_map[ind] >0.5)
          {            
            pt.x = ((float) x - (0.5 * (float) map_dimensions_.x)) * voxel_side_length_;
            pt.y = ((float)y - (0.5 * (float) map_dimensions_.y)) * voxel_side_length_;
            pt.z = 0;
            pt.intensity = 0.5;
            cloud.push_back(pt);
          }
        }
    }
    
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "odom";
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    ground_sdf_pub_.publish(cloud_msg);

  }

  void GPUVoxelsHSRServer::publishRVIZGroundSDFGrad(const std::vector<gpu_voxels::VectorSdfGrad> &sdf_grad_map) {

    visualization_msgs::MarkerArray marker_array;
    // marker_array.markers.resize(map_dimensions_.x * map_dimensions_.y);
    marker_array.markers.resize(36);

    // for (size_t x = 0; x < map_dimensions_.x; ++x){
    //   for (size_t y = 0; y < map_dimensions_.y; ++y){
    size_t i = 0;  
    size_t z = 2;
    for (size_t x = 290; x < 296; ++x){
      for (size_t y = 230; y < 236; ++y){
        // size_t ind = y * map_dimensions_.x + x;
        size_t ind = z * map_dimensions_.x * map_dimensions_.y + y * map_dimensions_.x + x;

        tf::Quaternion quaternion = tf::createQuaternionFromYaw(atan2( sdf_grad_map[ind].y, sdf_grad_map[ind].x));

        // std::cout << "Arrow " << i<< " \t x: " << sdf_grad_map_[ind].x << " \t y: " << sdf_grad_map_[ind].y << " \t Yaw: " << atan2(sdf_grad_map_[ind].x, sdf_grad_map_[ind].y) << std::endl;

        visualization_msgs::Marker marker;
        // Create a marker  
        marker.header.frame_id = "odom";
        marker.id = ind;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.03;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.pose.orientation.x = quaternion[0];
        marker.pose.orientation.y = quaternion[1];
        marker.pose.orientation.z = quaternion[2];        
        marker.pose.orientation.w = quaternion[3];
        marker.pose.position.x = ((float) x - (0.5 * (float) map_dimensions_.x)) * voxel_side_length_;
        marker.pose.position.y = ((float)y - (0.5 * (float) map_dimensions_.y)) * voxel_side_length_;
        marker.pose.position.z = 0;
        marker.ns = "gradient";
        marker.lifetime = ros::Duration(5);

        // Points are green
        marker.color.r = 1.0f;
        marker.color.a = 1.0;

        marker_array.markers[i] = marker;
        i++;
      }
    }
    ground_sdf_grad_pub_.publish(marker_array);
  
  }

  void GPUVoxelsHSRServer::publishRVIZConeRankings() {

    if (next_cone_robot_joints_.size()<=1)
    {
      return;
    }

    robot::JointValueMap robot_joints_ = next_cone_robot_joints_;

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(144);
    double x, y, z, w;


    // std::cout << "Publishing cone arrows" << std::endl;

    size_t ctr = 0;
    for (size_t i = 0; i < pan_deltas_.size(); i++){
      for (size_t j = 0; j < tilt_deltas_.size(); j++){
    // for (size_t i = 8; i < 9; i++){
    //   for (size_t j = 4; j < 5; j++){

        if (cone_costs_[ctr] < 0 ){
          ctr++;
          continue;
        }

        visualization_msgs::Marker marker;

        robot_joints_["head_pan_joint"] =  std::min(std::max(-3.0, (double) (next_cone_robot_joints_["head_pan_joint"] + pan_deltas_[i])), 1.7);
        robot_joints_["head_tilt_joint"] =  std::min(std::max(-1.20, (double) (next_cone_robot_joints_["head_tilt_joint"] + tilt_deltas_[j])), 0.1);
        robot_ptr_->setConfigurationNoPclUpdate(robot_joints_);
        KDL::Frame camera_pose = robot_ptr_->getLink("head_rgbd_sensor_link")->getPose();
        camera_pose.M.DoRotX(M_PI_2);
        camera_pose.M.DoRotZ(M_PI_2);
        camera_pose.M.GetQuaternion(x,y,z,w);


        marker.pose.position.x = camera_pose.p[0];
        marker.pose.position.y = camera_pose.p[1];
        marker.pose.position.z = camera_pose.p[2];

        // Create a marker  
        marker.header.frame_id = "odom";
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.05 * cone_costs_[ctr];
        marker.scale.y = 0.01 * cone_costs_[ctr];
        marker.scale.z = 0.01 * cone_costs_[ctr];
        marker.ns = "cone_arrows";
        marker.lifetime = ros::Duration(2);
        marker.color.a = 1.0;
        marker.id = ctr;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;        
        marker.pose.orientation.w = w;
        marker.pose.orientation.x = x;
        marker.pose.orientation.y = y;
        marker.pose.orientation.z = z;
        marker_array.markers[ctr] = marker;

        ctr++;

      }
    }

    cone_arrow_pub_.publish(marker_array);

  }

  void GPUVoxelsHSRServer::PointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    // std::cout << "PointcloudCallback" << std::endl;
    pointcloud_queue_.push(msg);
    CallbackSync();
  }

  double GPUVoxelsHSRServer::GetDistanceAndGradient(const Eigen::Vector3d &pos, Eigen::Vector3d &grad) const{
    // std::cout << "requesting GetDistanceAndGradient..." << std::endl;
    // std::cout << 
    gpu_voxels::Vector3f query_pos(pos[0] + (0.5 * (float) map_dimensions_.x * voxel_side_length_), 
                                  pos[1] + (0.5 * (float) map_dimensions_.y * voxel_side_length_), 
                                  pos[2]);


    query_pos.z = std::max((float) 0, query_pos.z);
    
    if( (float) query_pos.x < 0.0 || (float) query_pos.x >= (float) map_dimensions_.x * voxel_side_length_|| 
        (float) query_pos.y < 0.0 || (float) query_pos.y >= (float) map_dimensions_.y * voxel_side_length_|| 
        (float) query_pos.z < 0.0 || (float) query_pos.z >= (float) map_dimensions_.z * voxel_side_length_){
    

      std::cout << "Query out of bounds"<< std::endl;
      std::cout << "\t query_pos: \t " << "x:" << query_pos.x << "\t " << "y:" << query_pos.y << "\t "<< "z:" << query_pos.z << "\t "<< std::endl;
    
      grad[0] = 0;
      grad[1] = 0;
      grad[2] = 0;
      return 0.0;

    } 

    gpu_voxels::Vector3ui coords = gpu_voxels::voxelmap::mapToVoxels(voxel_side_length_, query_pos);

    // std::cout << "Coords: \t " << "x:" << coords.x << "\t " << "y:" << coords.y << "\t "<< "z:" << coords.z << "\t "<< std::endl;

    uint lin_ind = gpu_voxels::voxelmap::getVoxelIndexUnsigned(map_dimensions_, coords);

    grad[0] = sdf_grad_map_[lin_ind].x/voxel_side_length_;
    grad[1] = sdf_grad_map_[lin_ind].y/voxel_side_length_;
    grad[2] = sdf_grad_map_[lin_ind].z/voxel_side_length_;

    // if( (float) query_pos.z < 0.4){
    //   grad[2] = sdf_grad_map_[lin_ind].z;
    // }


    return sdf_grad_map_[lin_ind].sdf;
  }

  double GPUVoxelsHSRServer::GetTrilinearDistanceAndGradient(const Eigen::Vector3d &pos, Eigen::Vector3d &grad) const{

    gpu_voxels::Vector3f map_pos(pos[0] + (0.5 * (float) map_dimensions_.x * voxel_side_length_), 
                                  pos[1] + (0.5 * (float) map_dimensions_.y * voxel_side_length_), 
                                  pos[2]);


    map_pos.z = std::max((float) 0, map_pos.z);
    
    if( (float) map_pos.x < 0.0 || (float) map_pos.x >= (float) map_dimensions_.x * voxel_side_length_|| 
        (float) map_pos.y < 0.0 || (float) map_pos.y >= (float) map_dimensions_.y * voxel_side_length_|| 
        (float) map_pos.z < 0.0 || (float) map_pos.z >= (float) map_dimensions_.z * voxel_side_length_){
    

      // std::cout << "Query out of bounds"<< std::endl;
      // std::cout << "\t map_pos: \t " << "x:" << map_pos.x << "\t " << "y:" << map_pos.y << "\t "<< "z:" << map_pos.z << "\t "<< std::endl;
    
      grad[0] = 0;
      grad[1] = 0;
      grad[2] = 0;
      return 0.0;

    } 

    // In bounds
    
    gpu_voxels::Vector3f float_ind = map_pos/voxel_side_length_;
    gpu_voxels::Vector3ui l = gpu_voxels::voxelmap::mapToVoxels(voxel_side_length_, map_pos);
    // gpu_voxels::Vector3ui h = gpu_voxels::voxelmap::mapToVoxels(voxel_side_length_, map_pos + gpu_voxels::Vector3f(voxel_side_length_, voxel_side_length_, voxel_side_length_));
    gpu_voxels::Vector3ui h = l + gpu_voxels::Vector3ui(1, 1, 1);

    const float lx = (float) l.x, ly = (float) l.y, lz = (float) l.z; 
    const float hx = (float) h.x, hy = (float) h.y, hz = (float) h.z;

    // Now find the gradient


    grad[0] = (hy-float_ind.y)*(hz-float_ind.z) * (QueryDistance(h.x, l.y, l.z)-QueryDistance(l.x, l.y, l.z)) +
              (float_ind.y-ly)*(hz-float_ind.z) * (QueryDistance(h.x, h.y, l.z)-QueryDistance(l.x, h.y, l.z)) +
              (hy-float_ind.y)*(float_ind.z-lz) * (QueryDistance(h.x, l.y, h.z)-QueryDistance(l.x, l.y, h.z)) +
              (float_ind.y-ly)*(float_ind.z-lz) * (QueryDistance(h.x, h.y, h.z)-QueryDistance(l.x, h.y, h.z));

    grad[1] = (hx-float_ind.x)*(hz-float_ind.z) * (QueryDistance(l.x, h.y, l.z)-QueryDistance(l.x, l.y, l.z)) +
              (float_ind.x-lx)*(hz-float_ind.z) * (QueryDistance(h.x, h.y, l.z)-QueryDistance(h.x, l.y, l.z)) +
              (hx-float_ind.x)*(float_ind.z-lz) * (QueryDistance(l.x, h.y, h.z)-QueryDistance(l.x, l.y, h.z)) +
              (float_ind.x-lx)*(float_ind.z-lz) * (QueryDistance(h.x, h.y, h.z)-QueryDistance(h.x, l.y, h.z));

    grad[2] = (hx-float_ind.x)*(hy-float_ind.y) * (QueryDistance(l.x, l.y, h.z)-QueryDistance(l.x, l.y, l.z)) +
              (float_ind.x-lx)*(hy-float_ind.y) * (QueryDistance(h.x, l.y, h.z)-QueryDistance(h.x, l.y, l.z)) +
              (hx-float_ind.x)*(float_ind.y-ly) * (QueryDistance(l.x, h.y, h.z)-QueryDistance(l.x, h.y, l.z)) +
              (float_ind.x-lx)*(float_ind.y-ly) * (QueryDistance(h.x, h.y, h.z)-QueryDistance(h.x, h.y, l.z));
    
    
    // Now find the distance

    return (double)
        ((hx-float_ind.x)*(hy-float_ind.y)*(hz-float_ind.z)*QueryDistance(l.x, l.y, l.z) +
        (float_ind.x-lx)*(hy-float_ind.y)*(hz-float_ind.z)*QueryDistance(h.x, l.y, l.z) +
        (hx-float_ind.x)*(float_ind.y-ly)*(hz-float_ind.z)*QueryDistance(l.x, h.y, l.z) +
        (float_ind.x-lx)*(float_ind.y-ly)*(hz-float_ind.z)*QueryDistance(h.x, h.y, l.z) +
        (hx-float_ind.x)*(hy-float_ind.y)*(float_ind.z-lz)*QueryDistance(l.x, l.y, h.z) +
        (float_ind.x-lx)*(hy-float_ind.y)*(float_ind.z-lz)*QueryDistance(h.x, l.y, h.z) +
        (hx-float_ind.x)*(float_ind.y-ly)*(float_ind.z-lz)*QueryDistance(l.x, h.y, h.z) +
        (float_ind.x-lx)*(float_ind.y-ly)*(float_ind.z-lz)*QueryDistance(h.x, h.y, h.z));
  }

  double GPUVoxelsHSRServer::GetDistance(const Eigen::Vector3d &pos) const{

    gpu_voxels::Vector3f query_pos(pos[0] + (0.5 * (float) map_dimensions_.x * voxel_side_length_), 
                                  pos[1] + (0.5 * (float) map_dimensions_.y * voxel_side_length_), 
                                  pos[2]);


    query_pos.z = std::max((float) 0, query_pos.z);
    
    if( (float) query_pos.x < 0.0 || (float) query_pos.x >= (float) map_dimensions_.x * voxel_side_length_|| 
        (float) query_pos.y < 0.0 || (float) query_pos.y >= (float) map_dimensions_.y * voxel_side_length_|| 
        (float) query_pos.z < 0.0 || (float) query_pos.z >= (float) map_dimensions_.z * voxel_side_length_){

      std::cout << "Query out of bounds"<< std::endl;

      return 0.0;

    } 

    gpu_voxels::Vector3ui coords = gpu_voxels::voxelmap::mapToVoxels(voxel_side_length_, query_pos);

    uint lin_ind = gpu_voxels::voxelmap::getVoxelIndexUnsigned(map_dimensions_, coords);

    return sdf_grad_map_[lin_ind].sdf;
  }

  double GPUVoxelsHSRServer::QueryDistance(uint32_t xi, uint32_t yi, uint32_t zi) const{

    uint lin_ind = gpu_voxels::voxelmap::getVoxelIndexUnsigned(map_dimensions_, gpu_voxels::Vector3ui(xi, yi, zi));
    // return sdf_grad_map_[lin_ind].sdf;
    return sdf_map_[lin_ind];
  }

  double GPUVoxelsHSRServer::GetTrilinearDistance(const Eigen::Vector3d &pos) const{
    
    //  TODO - Is this needed?
    // pos.z = std::max((float) 0, pos.z);

    gpu_voxels::Vector3f map_pos(pos[0] + (0.5 * (float) map_dimensions_.x * voxel_side_length_), 
                                  pos[1] + (0.5 * (float) map_dimensions_.y * voxel_side_length_), 
                                  pos[2]);
    
    if(  map_pos.x < 0.0 ||  map_pos.x >= (float) map_dimensions_.x * voxel_side_length_ || 
         map_pos.y < 0.0 ||  map_pos.y >= (float) map_dimensions_.y * voxel_side_length_ || 
         map_pos.z < 0.0 ||  map_pos.z >= (float) map_dimensions_.z * voxel_side_length_ ){

      std::cout << "Query out of bounds"<< std::endl;

      return 0.0;

    } 

    // In bounds so now find the distance

    gpu_voxels::Vector3f float_ind = map_pos/voxel_side_length_;
    gpu_voxels::Vector3ui l = gpu_voxels::voxelmap::mapToVoxels(voxel_side_length_, map_pos);
    gpu_voxels::Vector3ui h = gpu_voxels::voxelmap::mapToVoxels(voxel_side_length_, map_pos + gpu_voxels::Vector3f(voxel_side_length_, voxel_side_length_, voxel_side_length_));

    const float lx = (float) l.x, ly = (float) l.y, lz = (float) l.z; 
    const float hx = (float) h.x, hy = (float) h.y, hz = (float) h.z;

    return (double)
        ((hx-float_ind.x)*(hy-float_ind.y)*(hz-float_ind.z)*QueryDistance(l.x, l.y, l.z) +
        (float_ind.x-lx)*(hy-float_ind.y)*(hz-float_ind.z)*QueryDistance(h.x, l.y, l.z) +
        (hx-float_ind.x)*(float_ind.y-ly)*(hz-float_ind.z)*QueryDistance(l.x, h.y, l.z) +
        (float_ind.x-lx)*(float_ind.y-ly)*(hz-float_ind.z)*QueryDistance(h.x, h.y, l.z) +
        (hx-float_ind.x)*(hy-float_ind.y)*(float_ind.z-lz)*QueryDistance(l.x, l.y, h.z) +
        (float_ind.x-lx)*(hy-float_ind.y)*(float_ind.z-lz)*QueryDistance(h.x, l.y, h.z) +
        (hx-float_ind.x)*(float_ind.y-ly)*(float_ind.z-lz)*QueryDistance(l.x, h.y, h.z) +
        (float_ind.x-lx)*(float_ind.y-ly)*(float_ind.z-lz)*QueryDistance(h.x, h.y, h.z));
  }

  void GPUVoxelsHSRServer::PoseCallback(const geometry_msgs::TransformStampedConstPtr &msg) {
    // std::cout << "PoseCallback" << std::endl;

    Vector3f camera_pos = Vector3f(msg->transform.translation.x,
                            msg->transform.translation.y ,
                            msg->transform.translation.z);

    tf::Quaternion q(msg->transform.rotation.x,
                    msg->transform.rotation.y,
                    msg->transform.rotation.z,
                    msg->transform.rotation.w);

    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    tf_ = Matrix4f::createFromRotationAndTranslation(Matrix3f::createFromRPY(roll, pitch, yaw), camera_pos);
    tf_ = tf_;
    // tf_ = tf_*T_D_B_*T_B_C_;

    tf_.a14 = tf_.a14  + map_dimensions_.x * voxel_side_length_ * 0.5f;
    tf_.a24 = tf_.a24 + map_dimensions_.y * voxel_side_length_ * 0.5f;

    cam_transform_queue_.push(std::make_tuple(msg->header.stamp, tf_));

  }

  void GPUVoxelsHSRServer::SaveSDFToFile(const std::string filepath){
        // Save the GPU-Voxels Occupancy to file
    std::ofstream savefile;
    savefile.open(filepath, std::fstream::out);

    for (size_t i = 0; i < sdf_map_.size(); i++)
    {
        savefile << sdf_map_[i];  

        if (i>0 && ((i+1) % (map_dimensions_.x * map_dimensions_.y) == 0))
        {
            savefile << '\n';  
        }
        else
        {
            savefile << ',';  
        }
        
    }
    savefile.close();
    LOGGING_INFO(Gpu_voxels, "File saved successfully" << endl);
  }

  void GPUVoxelsHSRServer::SaveOccupancyToFile(const std::string filepath){
        // Save the GPU-Voxels Occupancy to file
    std::ofstream savefile;
    savefile.open(filepath, std::fstream::out);

    for (size_t i = 0; i < occupancy_map_.size(); i++)
    {
        savefile << occupancy_map_[i];  

        if (i>0 && ((i+1) % (map_dimensions_.x * map_dimensions_.y) == 0))
        {
            savefile << '\n';  
        }
        else
        {
            savefile << ',';  
        }
        
    }
    savefile.close();
    LOGGING_INFO(Gpu_voxels, "File saved successfully" << endl);
  }

  // Constrain angles between -pi and pi
  float constrainAngle(float x){
      x = fmod(x + M_PI, 2*M_PI);
      if (x < 0)
          x += 2*M_PI;
      return x - M_PI;
  }

  float GPUVoxelsHSRServer::GetConeViewCost(robot::JointValueMap robot_joints){
    // Get the camera position for a given robot state
    // gvl_->setRobotConfiguration("hsrRobot", robot_joints);
    robot_ptr_->setConfigurationNoPclUpdate(robot_joints);

    KDL::Frame camera_pose = robot_ptr_->getLink("head_rgbd_sensor_link")->getPose();

    // Assumed that the base is on flat ground 
    // Camera theta = base theta + head pan
    float theta = constrainAngle(robot_joints["theta_joint"] + robot_joints["head_pan_joint"]); 
    float alpha = robot_joints["head_tilt_joint"]; 

    // criteria for being inside the cone
    gpu_voxels::Vector3f cam_pos(camera_pose.p[0], camera_pose.p[1], camera_pose.p[2]);
    gpu_voxels::Vector4f cam_fov(constrainAngle(theta - dtheta_/2), constrainAngle(theta + dtheta_/2), constrainAngle(alpha - dalpha_/2), constrainAngle(alpha + dalpha_/2));

    // Get the time cost

    // float cost = maintainedProbVoxmap_->getTotalConeCost(robotVoxmap_, cam_pos, cam_fov);
    float cost = maintainedProbVoxmap_->getTotalConeCost(cam_pos, cam_fov);

    // std::cout << "Cone time cost (avg): " <<  (float) cost / (float) (map_dimensions_.x * map_dimensions_.y * map_dimensions_.z) 
    //           << "\t" << "Cone traj cost (avg): " <<  (float) traj_cost / (float) (map_dimensions_.x * map_dimensions_.y * map_dimensions_.z)  << std::endl;
    // return (float) (cost + traj_cost) / (float) (map_dimensions_.x * map_dimensions_.y * map_dimensions_.z);
    return cost;
  }

  void GPUVoxelsHSRServer::SetConeFlags(robot::JointValueMap robot_joints){
    // gvl_->setRobotConfiguration("hsrRobot", robot_joints);
    robot_ptr_->setConfigurationNoPclUpdate(robot_joints);

    // Get the camera position for a given robot state
    KDL::Frame camera_pose = robot_ptr_->getLink("head_rgbd_sensor_link")->getPose();

    // Assumed that the base is on flat ground 
    // Camera theta = base theta + head pan
    float theta = constrainAngle(robot_joints["theta_joint"] + robot_joints["head_pan_joint"]); 
    float alpha = robot_joints["head_tilt_joint"]; 

    // These are the camera specific field of view parametera. TODO set these somewhere outside of member functions
    float dalpha = 1.1*M_PI_4;
    float dtheta = 1.3*M_PI_4;

    // criteria for being inside the cone
    gpu_voxels::Vector3f cam_pos(camera_pose.p[0], camera_pose.p[1], camera_pose.p[2]);
    gpu_voxels::Vector4f cam_fov(constrainAngle(theta - dtheta/2), constrainAngle(theta + dtheta/2), constrainAngle(alpha - dalpha/2), constrainAngle(alpha + dalpha/2));

    maintainedProbVoxmap_->setConeFlags(cam_pos, cam_fov);
  }

  void GPUVoxelsHSRServer::GetNBV(std::vector<robot::JointValueMap> robot_joints_vec, float (&nbv_joints)[2], const size_t current_ind){
    // LOGGING_INFO(Gpu_voxels, "GPUVoxelsHSRServer::GetNBV" << endl);

    // Camera limits 
    // tilt -1.570～0.523[rad]	-90~30[deg]
    // pan -3.839～1.745[rad]	-220～100[deg] 
    // size_t num_pan_primitives = 5;
    // size_t num_tilt_primitives = 3;
    // float tilt_range_div = (0.523 + 1.570)/ (float) num_tilt_primitives;
    // float pan_range_div = (3.839 + 1.745)/ (float) num_pan_primitives;
    size_t swept_vol_start_ind = std::min(current_ind + ind_delay_, robot_joints_vec.size() - 1); 

    timing::Timer nbv_timer("nbv_timer");
    gvl_->clearMap("myRobotMap");
    // gvl_->clearMap("myBitRobotMap");

    size_t counter =  robot_joints_vec.size() - 1 - swept_vol_start_ind;
    
    next_cone_robot_joints_ = robot_joints_vec[0];
    // next_cone_robot_joints_["x_joint"] = next_cone_robot_joints_["x_joint"] + (0.5 * (float) map_dimensions_.x * voxel_side_length_) ; 
    // next_cone_robot_joints_["y_joint"] = next_cone_robot_joints_["y_joint"] + (0.5 * (float) map_dimensions_.y * voxel_side_length_); 

    // Sweep trajectory volume
    if(robot_joints_vec.size() > 1){
      for (size_t i = robot_joints_vec.size() - 1; i >= swept_vol_start_ind; i--){
        robot_joints_vec[i]["x_joint"] = robot_joints_vec[i]["x_joint"] + (0.5 * (float) map_dimensions_.x * voxel_side_length_) ; 
        robot_joints_vec[i]["y_joint"] = robot_joints_vec[i]["y_joint"] + (0.5 * (float) map_dimensions_.y * voxel_side_length_); 
        // gvl_->setRobotConfiguration("hsrRobot", robot_joints_vec[i]);
        robot_ptr_->setConfiguration(robot_joints_vec[i]);

        BitVoxelMeaning v = BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + 1 + counter);
        // gvl_->insertRobotIntoMap("hsrRobot", "myRobotMap", v);
        robotVoxmap_->insertFixedVoxelMeaningMetaPointCloud(*robot_ptr_->getTransformedClouds(), v);

        // gvl_->insertRobotIntoMap("hsrRobot", "myBitRobotMap", v);
        counter--;
        // gvl_->visualizeMap("myBitRobotMap");
      }
    }
    else{
      robot_joints_vec[0]["x_joint"] = robot_joints_vec[0]["x_joint"] + (0.5 * (float) map_dimensions_.x * voxel_side_length_) ; 
      robot_joints_vec[0]["y_joint"] = robot_joints_vec[0]["y_joint"] + (0.5 * (float) map_dimensions_.y * voxel_side_length_); 
    }

    // Make current ind free/remove current volume
    robot_ptr_->setConfiguration(robot_joints_vec[current_ind]);
    robotVoxmap_->insertFixedVoxelMeaningMetaPointCloud(*robot_ptr_->getTransformedClouds(), BitVoxelMeaning(eBVM_FREE));


    // robotVoxmap_->getVoxelTrajCostsToHost(traj_step_map_);
    // timing::Timer costmap_calc_timer("costmap_calc_timer");
    maintainedProbVoxmap_->calculateCostmap(robotVoxmap_);
    // costmap_calc_timer.Stop();

    maintainedProbVoxmap_->getCostmapToHost(host_costmap_);

    robot::JointValueMap query_joint_map = robot_joints_vec[swept_vol_start_ind];
    std::vector<float> costs;

    // Primitives local to the current head pose
    // std::vector<float> pan_deltas = {-1.5, -1.25, -1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0, 1.5,};
    // std::vector<float> tilt_deltas = {-0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75};

    float max_cost =  -FLT_MAX;
    float min_cost =  FLT_MAX;
    for (size_t i = 0; i < pan_deltas_.size(); i++){
      float cost;
      // std::cout << " | " << std::endl;
      query_joint_map["head_pan_joint"] = robot_joints_vec[0]["head_pan_joint"] + pan_deltas_[i];
      for (size_t j = 0; j < tilt_deltas_.size(); j++){

        query_joint_map["head_tilt_joint"] = robot_joints_vec[0]["head_tilt_joint"] + tilt_deltas_[j];
        
        if (query_joint_map["head_pan_joint"] < -3.0 || query_joint_map["head_pan_joint"] > 1.7 ||
            query_joint_map["head_tilt_joint"] < -1.2 || query_joint_map["head_tilt_joint"] > 0.1)
        {
          cost = -FLT_MAX;
        }
        else{
          query_joint_map["head_pan_joint"] = std::min(std::max(-3.0, (double) query_joint_map["head_pan_joint"] ), 1.7);
          query_joint_map["head_tilt_joint"] = std::min(std::max(-1.20, (double) query_joint_map["head_tilt_joint"] ), 0.1);
          timing::Timer single_view_cost_timer("single_view_cost_timer");
          cost = this->GetConeViewCost(query_joint_map);
          single_view_cost_timer.Stop();
          max_cost = std::max(max_cost, cost);
          min_cost = std::min(min_cost, cost);
        }

        costs.push_back(cost);
        // std::cout << cost << " | ";

      } 
      // std::cout << std::endl;
      // std::cout << "---------------------"<< std::endl;
    }

    int min_cost_idx = std::max_element( costs.begin(), costs.end()) - costs.begin();
    size_t pan_ind = floor((float) min_cost_idx / (float) tilt_deltas_.size());
    size_t tilt_ind = min_cost_idx % tilt_deltas_.size();
    
    // float pan_clamped = std::max(-0.7f, (float) std::min(pan_deltas[pan_ind], 0.7f));
    // pan_clamped += robot_joints_vec[0]["head_pan_joint"];
    // float tilt_clamped = std::max(-0.7f, (float) std::min(tilt_deltas[pan_ind], 0.7f));
    // tilt_clamped += robot_joints_vec[0]["head_tilt_joint"];
    // Clamp to the max deviation we want per timestep
    // Clamp to the absolute max and min of the joints (or what we set it to at least)
    // nbv_joints[0] =  std::min(std::max(-3.5f, pan_clamped), 1.6f);
    // nbv_joints[1] =  std::min(std::max(-1.20f, tilt_clamped), 0.1f);

    // Clamp to the absolute max and min of the joints (or what we set it to at least)
    nbv_joints[0] =  std::min(std::max(-3.0, (double) (robot_joints_vec[0]["head_pan_joint"] + pan_deltas_[pan_ind])), 1.7);
    nbv_joints[1] =  std::min(std::max(-1.20, (double) (robot_joints_vec[0]["head_tilt_joint"] + tilt_deltas_[tilt_ind])), 0.1);
    nbv_timer.Stop();
    
    cone_costs_ = costs;
    for(int i=0;i<cone_costs_.size();++i){
      cone_costs_[i] = (cone_costs_[i] - min_cost) / (max_cost - min_cost);
    }
        // float pan_joint = -3.839 + i * pan_range_div; 
    // float tilt_joint = -1.570 + j * tilt_range_div;


    // std::cout << "Head Change: \t " << pan_deltas[pan_ind] << "\t " << tilt_deltas[tilt_ind] << std::endl;
    // std::cout << "Requesting: \t pan: " << robot_joints_vec[0]["head_pan_joint"] << "-->" << nbv_joints[0] << std::endl;
    // std::cout << "            \t tilt: " << robot_joints_vec[0]["head_tilt_joint"] << "-->" << nbv_joints[1] << std::endl;

    // robot_joints_vec[nbv_time_ind]["head_pan_joint"] = nbv_joints[0];
    // robot_joints_vec[nbv_time_ind]["head_tilt_joint"] = nbv_joints[1];
    // this->SetConeFlags(robot_joints_vec[nbv_time_ind]);

    // maintainedProbVoxmap_->getVoxelFlagsToHost(flag_map_);
    // publishRVIZVoxelFlags(flag_map_);
  }

} // namespace 