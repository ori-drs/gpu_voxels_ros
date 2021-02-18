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

    node_.param<float>("voxel_side_length", voxel_side_length_, 0.05f);
    node_.param<int>("map_size_x", (int&) map_dimensions_.x, 448);
    node_.param<int>("map_size_y", (int&) map_dimensions_.y, 448);
    node_.param<int>("map_size_z", (int&) map_dimensions_.z, 128);

    // node_.param<float>("voxel_side_length", voxel_side_length_, 0.02f);
    // node_.param<int>("map_size_x", (int&) map_dimensions_.x, 512);
    // node_.param<int>("map_size_y", (int&) map_dimensions_.y, 512);
    // node_.param<int>("map_size_z", (int&) map_dimensions_.z, 128);

    map_pub_ = node.advertise<pcl::PointCloud<pcl::PointXYZI> >("gpu_voxels_pointcloud", 1, true);
    ground_sdf_pub_ = node.advertise<pcl::PointCloud<pcl::PointXYZI> >("gpu_voxels_ground_sdf", 1, true);
    ground_sdf_grad_pub_ = node.advertise<visualization_msgs::MarkerArray>("gpu_voxels_ground_sdf_grad", 2000, true);

    transform_sub_ = node.subscribe(transform_topic_, 10, &GPUVoxelsHSRServer::PoseCallback, this);
    pcl_sub_ = node.subscribe(pcl_topic_, 10, &GPUVoxelsHSRServer::PointcloudCallback, this);

    // Generate a GPU-Voxels instance:
    gvl_ = gpu_voxels::GpuVoxels::getInstance();
    gvl_->initialize(map_dimensions_.x, map_dimensions_.y, map_dimensions_.z, voxel_side_length_);
    gvl_->addMap(MT_SIGNED_DISTANCE_VOXELMAP, "pbaDistanceVoxmap");
    gvl_->addMap(MT_PROBAB_VOXELMAP, "maintainedProbVoxmap");
    gvl_->addMap(MT_DISTANCE_VOXELMAP, "pbaDistanceVoxmapVisual");

    // gvl_->addMap(MT_COUNTING_VOXELLIST, "countingVoxelList");
    // gvl_->addMap(MT_COUNTING_VOXELLIST, "countingVoxelListFiltered");
    // gvl_->addMap(MT_PROBAB_VOXELMAP, "erodeTempVoxmap1");
    // gvl_->addMap(MT_PROBAB_VOXELMAP, "erodeTempVoxmap2");

    pbaDistanceVoxmap_ = dynamic_pointer_cast<DistanceVoxelMap>(gvl_->getMap("pbaDistanceVoxmap"));
    pbaInverseDistanceVoxmap_ = dynamic_pointer_cast<DistanceVoxelMap>(gvl_->getMap("pbaDistanceVoxmapInverse"));
    maintainedProbVoxmap_ = dynamic_pointer_cast<ProbVoxelMap>(gvl_->getMap("maintainedProbVoxmap"));
    pbaDistanceVoxmapVisual_ = dynamic_pointer_cast<DistanceVoxelMap>(gvl_->getMap("pbaDistanceVoxmapVisual"));

    // countingVoxelList_ = dynamic_pointer_cast<CountingVoxelList>(gvl_->getMap("countingVoxelList"));
    // erodeTempVoxmap1_ = dynamic_pointer_cast<ProbVoxelMap>(gvl_->getMap("erodeTempVoxmap1"));
    // erodeTempVoxmap2_ = dynamic_pointer_cast<ProbVoxelMap>(gvl_->getMap("erodeTempVoxmap2"));
    // countingVoxelListFiltered_ = dynamic_pointer_cast<CountingVoxelList>(gvl_->getMap("countingVoxelListFiltered"));

    signedDistanceMap_ = dynamic_pointer_cast<InheritSignedDistanceVoxelMap>(pbaDistanceVoxmap_);

    sdf_grad_map_ = std::vector<gpu_voxels::VectorSdfGrad>(pbaDistanceVoxmap_->getVoxelMapSize());
    sdf_map_ = std::vector<float>(pbaDistanceVoxmap_->getVoxelMapSize());
    occupancy_map_ = std::vector<int>(pbaDistanceVoxmap_->getVoxelMapSize());

    maintainedProbVoxmap_->clearMap();

    std::cout << "HSR Server Ready" << std::endl;
  }

  void GPUVoxelsHSRServer::CallbackSync(){

    timing::Timer sync_callback_timer("CallbackSync");

    ros::Time pcl_time;
    double time_delay = 2e-3;
    // int filter_threshold = 1;
    // float erode_threshold = 0.0f;
    // std::cout << "Remove voxels containing fewer points than: " << filter_threshold << std::endl;

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

        // signedDistanceMap_->clearMaps();
        pbaDistanceVoxmap_->clearMap();

        // countingVoxelList_->clearMap();
        // countingVoxelListFiltered_->clearMap();
        // erodeTempVoxmap1_->clearMap();
        // erodeTempVoxmap2_->clearMap();
        // maintainedProbVoxmap_->clearMap();
        
        // countingVoxelList_->insertPointCloud(my_point_cloud_, eBVM_OCCUPIED);
        // countingVoxelListFiltered_->merge(countingVoxelList_);
        // countingVoxelListFiltered_->remove_underpopulated(filter_threshold);
        // gvl_->visualizeMap("countingVoxelList");
        // gvl_->visualizeMap("countingVoxelListFiltered");

        // erodeTempVoxmap1_->merge(countingVoxelListFiltered_);
        // gvl_->visualizeMap("erodeTempVoxmap1");
        // erodeTempVoxmap1_->erodeLonelyInto(*erodeTempVoxmap2_); //erode only "lonely voxels" without occupied neighbors
        
        // gvl_->visualizeMap("erodeTempVoxmap2");

        // // maintainedProbVoxmap_->mergeOccupied(erodeTempVoxmap2_);
        // signedDistanceMap_->occupancyMerge(erodeTempVoxmap2_, 0.95, 0.94999);


        // maintainedProbVoxmap_->insertSensorData<BIT_VECTOR_LENGTH>(my_point_cloud_, camera_pos, true, false, eBVM_OCCUPIED, NULL);
        maintainedProbVoxmap_->insertClippedSensorData<BIT_VECTOR_LENGTH>(my_point_cloud_, camera_pos, true, false, eBVM_OCCUPIED, 
                                                                          min_ray_length_, max_ray_length_, NULL, remove_floor_);

        signedDistanceMap_->occupancyMerge(maintainedProbVoxmap_, 0.75, 0.74999);

        // signedDistanceMap_->parallelBanding3DMark();
        signedDistanceMap_->parallelBanding3DUnsigned();
        // signedDistanceMap_->parallelBanding3DSigned();

        update_esdf_timer.Stop();

        timing::Timer transfer_timer("HostRetrieval");

        // std::cout << "Retrieving gradients and SDF" << std::endl;
        // signedDistanceMap_->getSignedDistancesAndGradientsToHost(sdf_grad_map_);
        // signedDistanceMap_->getSignedDistancesToHost(sdf_map_);
        // pbaDistanceVoxmap_->getUnsignedDistancesToHost(sdf_map_);
        // pbaDistanceVoxmap_->getOccupancyToHost(occupancy_map_);



        // pbaDistanceVoxmap_->getSignedDistancesAndGradientsToHost(pbaInverseDistanceVoxmap_, sdf_grad_map_);
        // pbaDistanceVoxmap_->getSignedDistancesToHost(pbaInverseDistanceVoxmap_, sdf_map_);
        pbaDistanceVoxmap_->getUnsignedDistancesToHost(sdf_map_);
        // std::cout << "Found gradients and SDF" << std::endl;

        transfer_timer.Stop();
        sync_callback_timer.Stop();
        pointcloud_queue_.pop();

        // timing::Timing::Print(std::cout);
        // pbaDistanceVoxmapVisual_->clone(*(pbaDistanceVoxmap_.get()));
        // gvl_->visualizeMap("pbaDistanceVoxmapVisual");        
        
        // publishRVIZOccupancy(sdf_grad_map_);
        // publishRVIZGroundSDF(sdf_grad_map_);

        publishRVIZGroundSDF(sdf_map_);
        publishRVIZOccupancy(sdf_map_);
        // publishRVIZOccupancy(occupancy_map_);
        // std::cout << "Finished publishing" << std::endl;

    }
    // publishRVIZGroundSDFGrad(sdf_grad_map_);
  }

  void GPUVoxelsHSRServer::publishRVIZOccupancy(const std::vector<int> &occupancy_map) {
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    for (size_t x = 0; x <= map_dimensions_.x; ++x)
      for (size_t y = 0; y <= map_dimensions_.y; ++y)
        for (size_t z = 0; z <= map_dimensions_.z; ++z) {
          size_t ind = z * map_dimensions_.x * map_dimensions_.y + y * map_dimensions_.x + x;
          if (occupancy_map[ind] == 1) {
            pt.x = ((float) x + 0.5 * (float) map_dimensions_.x) * voxel_side_length_;
            pt.y = ((float)y + 0.5 * (float) map_dimensions_.y) * voxel_side_length_;
            pt.z = (float)z + 0.5 * (float) voxel_side_length_;
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
    

      std::cout << "Query out of bounds"<< std::endl;
      std::cout << "\t map_pos: \t " << "x:" << map_pos.x << "\t " << "y:" << map_pos.y << "\t "<< "z:" << map_pos.z << "\t "<< std::endl;
    
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
} // namespace 