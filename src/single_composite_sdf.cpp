// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the gpu_voxels_ros package.
// © Copyright 2022, Mark Finean 
// 
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the followingdisclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation 
// and/or other materials provided with the distribution.
// 
// 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software 
// without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS 
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// -- END LICENSE BLOCK ------------------------------------------------
//----------------------------------------------------------------------
/*!\file
 *
 * \author  Mark Finean
 * \date    2020-09-24 
 */
//----------------------------------------------------------------------

#include <gpu_voxels_ros/single_composite_sdf.h>
#include <gpu_voxels_ros/timing.h>
namespace gpu_voxels_ros{

  SingleCompositeSDF::SingleCompositeSDF(ros::NodeHandle& node) {
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
    node_.param<std::string>("traj_pred_topic", traj_pred_topic_, "/traj_predictions");
    node_.param<std::string>("pcl_topic", pcl_topic_, "/hsrb/head_rgbd_sensor/depth_registered/rectified_points");
    node_.param<std::string>("sensor_frame", sensor_frame_, "head_rgbd_sensor_rgb_frame");

    node_.param<float>("voxel_side_length", voxel_side_length_, 0.05f);
    node_.param<int>("map_size_x", (int&) map_dimensions_.x, 256);
    node_.param<int>("map_size_y", (int&) map_dimensions_.y, 256);
    node_.param<int>("map_size_z", (int&) map_dimensions_.z, 64);

    std::cout << "Using voxelmap of size (" << map_dimensions_.x << "," 
                                          << map_dimensions_.x << ", " 
                                          << map_dimensions_.z << ")" << std::endl;

    num_sdfs_ = 20;

    unclean_map_pub_ = node.advertise<pcl::PointCloud<pcl::PointXYZI> >("unclean_gpu_voxels_pointcloud", 1, true);
    map_pub_ = node.advertise<pcl::PointCloud<pcl::PointXYZI> >("gpu_voxels_pointcloud", 1, true);
    ground_sdf_pub_ = node.advertise<pcl::PointCloud<pcl::PointXYZI> >("gpu_voxels_ground_sdf", 1, true);
    update_time_pub_ = node.advertise<pcl::PointCloud<pcl::PointXYZI> >("gpu_voxels_update_times", 1, true);
    ground_sdf_grad_pub_ = node.advertise<visualization_msgs::MarkerArray>("gpu_voxels_ground_sdf_grad", 2000, true);
    cone_flag_pub_ = node.advertise<pcl::PointCloud<pcl::PointXYZI> >("gpu_voxels_cone_flags", 1, true);
    traj_sweep_pub_ = node.advertise<pcl::PointCloud<pcl::PointXYZI> >("gpu_voxels_traj_steps", 1, true);
    distance_field_2d_pub_= node.advertise<std_msgs::Float32MultiArray>("distance_field_2d", 1, true);
    costmap_pub_ = node.advertise<pcl::PointCloud<pcl::PointXYZI> >("gpu_voxels_costmap", 1, true);
    ground2dsdf_pub_ = node.advertise<pcl::PointCloud<pcl::PointXYZI> >("distancefield_2d_pcl", 1, true);


    transform_sub_ = node.subscribe(transform_topic_, 10, &SingleCompositeSDF::PoseCallback, this);
    pcl_sub_ = node.subscribe(pcl_topic_, 1, &SingleCompositeSDF::PointcloudCallback, this);
    traj_pred_sub_ = node.subscribe(traj_pred_topic_, 1, &SingleCompositeSDF::HumanTrajectoryPredictionCallback, this);

    // Generate a GPU-Voxels instance:
    gvl_ = gpu_voxels::GpuVoxels::getInstance();
    gvl_->initialize(map_dimensions_.x, map_dimensions_.y, map_dimensions_.z, voxel_side_length_);
    gvl_->addMap(MT_SIGNED_DISTANCE_VOXELMAP, "pbaDistanceVoxmap");
    gvl_->addMap(MT_PROBAB_VOXELMAP, "maintainedProbVoxmap");
    gvl_->addMap(MT_DISTANCE_VOXELMAP, "pbaDistanceVoxmapVisual");
    gvl_->addMap(MT_PROBAB_VOXELMAP, "myRobotMap");
    gvl_->addMap(MT_PROBAB_VOXELMAP, "cleanVoxmap");
    gvl_->addMap(MT_PROBAB_VOXELMAP, "cleanVoxmapVisual");

    distvoxelmap_2d_ = boost::shared_ptr<DistanceVoxelMap> (new DistanceVoxelMap(Vector3ui(map_dimensions_.x, map_dimensions_.y, 1), voxel_side_length_, MT_DISTANCE_VOXELMAP));

    // Add the robot model
    gvl_->addRobot("hsrRobot", "hsr/originals/mod_hsrb.urdf", true);
    std::cout << "Added HSR Urdf model" << std::endl;

    robot_ptr_ = dynamic_pointer_cast<gpu_voxels::robot::UrdfRobot>(gvl_->getRobot("hsrRobot"));

    robotVoxmap_ = dynamic_pointer_cast<ProbVoxelMap>(gvl_->getMap("myRobotMap"));

    pbaDistanceVoxmap_ = dynamic_pointer_cast<DistanceVoxelMap>(gvl_->getMap("pbaDistanceVoxmap"));
    pbaInverseDistanceVoxmap_ = dynamic_pointer_cast<DistanceVoxelMap>(gvl_->getMap("pbaDistanceVoxmapInverse"));
    maintainedProbVoxmap_ = dynamic_pointer_cast<ProbVoxelMap>(gvl_->getMap("maintainedProbVoxmap"));
    pbaDistanceVoxmapVisual_ = dynamic_pointer_cast<DistanceVoxelMap>(gvl_->getMap("pbaDistanceVoxmapVisual"));
    cleanVoxmap_ = dynamic_pointer_cast<ProbVoxelMap>(gvl_->getMap("cleanVoxmap"));

    sdf_map_ = std::vector<float>(pbaDistanceVoxmap_->getVoxelMapSize());
    unclean_sdf_map_ = std::vector<float>(pbaDistanceVoxmap_->getVoxelMapSize());

    host_costmap_ = std::vector<float>(pbaDistanceVoxmap_->getVoxelMapSize());
    host_2d_dist_ = std::vector<float>(distvoxelmap_2d_->getVoxelMapSize());
    
    maintainedProbVoxmap_->clearMap();
    maintainedProbVoxmap_->resetTimeSteps();

    // Generate the human sdf cylinder 
    human_dims_ = Vector3ui(64,64,map_dimensions_.z);
    Vector3f cylinder_center(64 * 0.5 * voxel_side_length_,
                            64 * 0.5 * voxel_side_length_, 
                            1.0);

 
    std::vector<Vector3f> cylinder_points = gpu_voxels::geometry_generation::createCylinderOfPoints(cylinder_center, 0.3f, 2.0f, voxel_side_length_);
    std::vector<Vector3f> all_points = gpu_voxels::geometry_generation::createBoxOfPoints(Vector3f(0,0,0), 
                                                                                          Vector3f(human_dims_.x * voxel_side_length_, 
                                                                                                  human_dims_.y * voxel_side_length_ , 
                                                                                                  human_dims_.x * voxel_side_length_), 
                                                                                          voxel_side_length_);

    human_shared_ptr_ = boost::shared_ptr<DistanceVoxelMap>(new DistanceVoxelMap(human_dims_, voxel_side_length_, MT_DISTANCE_VOXELMAP));
    // human_inverse_shared_ptr_ = boost::shared_ptr<DistanceVoxelMap>(new DistanceVoxelMap(human_dims_, voxel_side_length_, MT_DISTANCE_VOXELMAP));

    // humanSignedDistanceMap_ = boost::shared_ptr<SignedDistanceVoxelMap>(new SignedDistanceVoxelMap(human_shared_ptr_, human_inverse_shared_ptr_, human_dims_, voxel_side_length_));
    
    signedDistanceMap_ = dynamic_pointer_cast<InheritSignedDistanceVoxelMap>(pbaDistanceVoxmap_);

    human_shared_ptr_->insertPointCloud(cylinder_points, eBVM_OCCUPIED);
    // human_inverse_shared_ptr_->insertPointCloud(all_points, eBVM_OCCUPIED);
    // human_inverse_shared_ptr_->insertPointCloud(cylinder_points, eBVM_FREE);

    human_shared_ptr_->parallelBanding3D();
    // humanSignedDistanceMap_->parallelBanding3D();

    pbaDistanceVoxmap_->addHuman(human_shared_ptr_, human_dims_);
    // pbaDistanceVoxmap_->addSignedHuman(humanSignedDistanceMap_, human_dims_);

    std::cout << "HSR Server Ready" << std::endl;
  }

  void SingleCompositeSDF::CallbackSync(){

    // std::cout << "Callback executed" << std::endl;

    timing::Timer sync_callback_timer("CallbackSync");

    ros::Time pcl_time;
    double time_delay = 2e-2;
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

        // std::cout << "Latency start of composite SDF update: " << (ros::Time::now() - pcl_time).toSec() << std::endl;

        sensor_msgs::PointCloud2::ConstPtr msg = pointcloud_queue_.front();
        pcl::fromROSMsg(*msg, cloud_);
        
        if ((int) cloud_.size()==0) {
              pointcloud_queue_.pop();
              continue;
        }

        Vector3f camera_pos = Vector3f(sync_tf_.a14, sync_tf_.a24, sync_tf_.a34)/sync_tf_.a44;

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


        pbaDistanceVoxmap_->clearMap();
        distvoxelmap_2d_->clearMap();
        // pbaDistanceVoxmapVisual_->clearMap();

        timing::Timer pcl_timer("PCL insertion");
        maintainedProbVoxmap_->insertClippedSensorData<BIT_VECTOR_LENGTH>(my_point_cloud_, camera_pos, true, false, eBVM_OCCUPIED, 
                                                                          min_ray_length_, max_ray_length_, NULL, remove_floor_);
                                                                    
        pcl_timer.Stop();
        // Get a 2D distance field for use in human trajectory prediction

        // maintainedProbVoxmap_->erodeLonelyInto(*cleanVoxmap_); //erode only "lonely voxels" without occupied neighbors
        timing::Timer erosion_timer("Erosion");
        cleanVoxmap_->clearMap();
        maintainedProbVoxmap_->erodeInto(*cleanVoxmap_, 0.35, 0.75); // 0.08 requires at least 2 surroundings
        erosion_timer.Stop();



        // signedDistanceMap_->occupancyMerge(cleanVoxmap_, 0.75, 0.74999);
        
        timing::Timer dist_2d_compute_timer("DistanceField2DCompute");
        // distvoxelmap_2d_->merge2DOccupied(maintainedProbVoxmap_, Vector3ui(0),  0.75, 2);
        distvoxelmap_2d_->merge2DOccupied(cleanVoxmap_, Vector3ui(0),  0.75, 2);
        distvoxelmap_2d_->parallelBanding3D(1, 1, 4, PBA_DEFAULT_M1_BLOCK_SIZE, PBA_DEFAULT_M2_BLOCK_SIZE, PBA_DEFAULT_M3_BLOCK_SIZE, 1);
        // std::cout << "distvoxelmap_2d_->getUnsignedDistancesToHost" << std::endl;
        distvoxelmap_2d_->getUnsignedDistancesToHost(host_2d_dist_); 
        // cudaDeviceSynchronize();
        dist_2d_compute_timer.Stop();
        
        // timing::Timer composite_timer("Compositing");

        timing::Timer update_esdf_timer("UpdateESDF");
        timing::Timer pba_timer("pba");
        // pbaDistanceVoxmapVisual_->mergeOccupied(maintainedProbVoxmap_, Vector3ui(0), 0.75);
        // pbaDistanceVoxmapVisual_->parallelBanding3D();

        signedDistanceMap_->occupancyMerge(cleanVoxmap_, 0.75, 0.74999);
        signedDistanceMap_->parallelBanding3DUnsigned();
        // cudaDeviceSynchronize();
        pba_timer.Stop();   

        update_esdf_timer.Stop();

        // timing::Timer static_sdf_timer("StaticSDF");
      
        // static_sdf_timer.Stop();


        pbaDistanceVoxmap_->computeStaticSDF();
        cudaDeviceSynchronize();


        if(human_traj_latest_){
          std::lock_guard<std::mutex> lock(traj_msg_mutex_);

          size_t num_poses = human_traj_latest_->poses.size();

          if (num_poses > 0)
          {
            pbaDistanceVoxmap_->compositeSDFReinit();

            float human_x = human_traj_latest_->poses[0].position.x;
            float human_y = human_traj_latest_->poses[0].position.y;
            int human_xi = round(human_x/voxel_side_length_);
            int human_yi = round(human_y/voxel_side_length_);

            cylinder_base_corner_ = Vector3ui(map_dimensions_.x/2+human_xi - 32 ,map_dimensions_.y/2 +human_yi -32 ,0);
            pbaDistanceVoxmap_->getCompositeSDF(human_shared_ptr_->getVoxelMapSize(), human_dims_, cylinder_base_corner_);
            pbaDistanceVoxmap_->getCompositeSDFToHost(sdf_map_);

          } // if num_poses > 0 
          else{
            pbaDistanceVoxmap_->compositeSDFReinit();
            pbaDistanceVoxmap_->getCompositeSDFToHost(sdf_map_);
          }
        }
        else{
          pbaDistanceVoxmap_->compositeSDFReinit();
          pbaDistanceVoxmap_->getCompositeSDFToHost(sdf_map_);
        }

        // transfer_timer.Stop();
        cudaDeviceSynchronize();
        sync_callback_timer.Stop();
        pointcloud_queue_.pop();


        timing::Timer publish_timer("publishing");
        publishRVIZGroundSDF(sdf_map_);
        publishRVIZOccupancy(sdf_map_);
        // publishRVIZUncleanOccupancy(unclean_sdf_map_);
        publish2DDistanceField(host_2d_dist_);
        // publish2DDistanceFieldImage(host_2d_dist_);

        publish_timer.Stop();

        // timing::Timing::Print(std::cout);

    }

  }

  float SingleCompositeSDF::getPercentageMapExplored() const{
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

  void SingleCompositeSDF::publish2DDistanceField(const std::vector<float> &distance_field_2d) {
    // std::cout << "publish2DDistanceField" << std::endl;

    // map_dimensions_.x,map_dimensions_.y
		std_msgs::Float32MultiArray array;
    array.data = distance_field_2d;
    // array.layout = distance_field_2d;
    distance_field_2d_pub_.publish(array);
  }

  void SingleCompositeSDF::publish2DDistanceFieldImage(const std::vector<float> &distance_field_2d) {
    pcl::PointXYZI pt;
    pcl::PointCloud<pcl::PointXYZI> cloud;

    for (size_t x = 0; x < map_dimensions_.x; ++x){
      for (size_t y = 0; y < map_dimensions_.y; ++y){
          size_t ind = y * map_dimensions_.x + x;

          if (distance_field_2d[ind] <= 0.5)
          {            
            pt.x = ((float) x - (0.5 * (float) map_dimensions_.x)) * voxel_side_length_;
            pt.y = ((float)y - (0.5 * (float) map_dimensions_.y)) * voxel_side_length_;
            pt.z = 0;
            pt.intensity = distance_field_2d[ind];
            cloud.push_back(pt);
          }

          if (distance_field_2d[ind] >0.5)
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
    ground2dsdf_pub_.publish(cloud_msg);

  }

  void SingleCompositeSDF::publishRVIZVoxelFlags(const std::vector<bool> &flag_map) {
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

  void SingleCompositeSDF::publishRVIZCostmap(const std::vector<float> &costmap) {
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

  void SingleCompositeSDF::publishRVIZUpdateTimes(const std::vector<uint16_t> &time_map, uint16_t threshold) {
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

  void SingleCompositeSDF::publishRVIZTrajSweepOccupancy(const std::vector<int> &occupancy_map) {
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

  void SingleCompositeSDF::publishRVIZOccupancy(const std::vector<int> &occupancy_map) {
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

  void SingleCompositeSDF::publishRVIZOccupancy(const std::vector<float> &sdf_map) {
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

  void SingleCompositeSDF::publishRVIZUncleanOccupancy(const std::vector<float> &sdf_map) {
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
    unclean_map_pub_.publish(cloud_msg);
  }


  void SingleCompositeSDF::publishRVIZOccupancy(const std::vector<gpu_voxels::VectorSdfGrad> &sdf_grad_map) {
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

  void SingleCompositeSDF::publishRVIZGroundSDF(const std::vector<gpu_voxels::VectorSdfGrad> &sdf_grad_map) {
  
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

  void SingleCompositeSDF::publishRVIZGroundSDF(const std::vector<float> &sdf_map) {
  
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

  void SingleCompositeSDF::publishRVIZGroundSDFGrad(const std::vector<gpu_voxels::VectorSdfGrad> &sdf_grad_map) {

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

  void SingleCompositeSDF::PointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    // std::cout << "PointcloudCallback" << std::endl;
    pointcloud_queue_.push(msg);
    CallbackSync();
  }

  void SingleCompositeSDF::HumanTrajectoryPredictionCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(traj_msg_mutex_);
    human_traj_latest_ = msg;
  }

  double SingleCompositeSDF::GetDistanceAndGradient(const Eigen::Vector3d &pos, Eigen::Vector3d &grad) const{
    // std::cout << "requesting GetDistanceAndGradient..." << std::endl;
    // std::cout << 
    gpu_voxels::Vector3f query_pos(pos[0] + (0.5 * (float) map_dimensions_.x * voxel_side_length_), 
                                  pos[1] + (0.5 * (float) map_dimensions_.y * voxel_side_length_), 
                                  pos[2]);


    query_pos.z = std::max((float) 0, query_pos.z);
    
    if( (float) query_pos.x < 0.0 || (float) query_pos.x >= (float) map_dimensions_.x * voxel_side_length_|| 
        (float) query_pos.y < 0.0 || (float) query_pos.y >= (float) map_dimensions_.y * voxel_side_length_|| 
        (float) query_pos.z < 0.0 || (float) query_pos.z >= (float) map_dimensions_.z * voxel_side_length_){
    

      // std::cout << "Query out of bounds"<< std::endl;
      // std::cout << "\t query_pos: \t " << "x:" << query_pos.x << "\t " << "y:" << query_pos.y << "\t "<< "z:" << query_pos.z << "\t "<< std::endl;
    
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

  double SingleCompositeSDF::GetTrilinearDistanceAndGradient(const Eigen::Vector3d &pos, Eigen::Vector3d &grad) const{

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

  double SingleCompositeSDF::GetDistance(const Eigen::Vector3d &pos) const{

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

  double SingleCompositeSDF::QueryDistance(uint32_t xi, uint32_t yi, uint32_t zi) const{
    // std::cout << "QueryDistance" << std::endl;
    uint lin_ind = gpu_voxels::voxelmap::getVoxelIndexUnsigned(map_dimensions_, gpu_voxels::Vector3ui(xi, yi, zi));
    // return sdf_grad_map_[lin_ind].sdf;
    return sdf_map_[lin_ind];
  }

  double SingleCompositeSDF::GetTrilinearDistance(const Eigen::Vector3d &pos) const{
    
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

  void SingleCompositeSDF::PoseCallback(const geometry_msgs::TransformStampedConstPtr &msg) {
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

  void SingleCompositeSDF::SaveSDFToFile(const std::string filepath){
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

  void SingleCompositeSDF::SaveOccupancyToFile(const std::string filepath){
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

  float SingleCompositeSDF::GetConeViewCost(robot::JointValueMap robot_joints){
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

  void SingleCompositeSDF::SetConeFlags(robot::JointValueMap robot_joints){
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

  void SingleCompositeSDF::GetNBV(std::vector<robot::JointValueMap> robot_joints_vec, float (&nbv_joints)[2], const size_t current_ind){
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

    // maintainedProbVoxmap_->getCostmapToHost(host_costmap_);

    robot::JointValueMap query_joint_map = robot_joints_vec[swept_vol_start_ind];
    std::vector<float> costs;

    // Primitives local to the current head pose
    // std::vector<float> pan_deltas = {-1.5, -1.25, -1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0, 1.5,};
    // std::vector<float> tilt_deltas = {-0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75};
    std::vector<float> pan_deltas = {-3.14, -2.0, -1.5, -1.25, -1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.14};
    std::vector<float> tilt_deltas = {-1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0};
    for (size_t i = 0; i < pan_deltas.size(); i++){
      float cost;
      // std::cout << " | " << std::endl;
      query_joint_map["head_pan_joint"] = robot_joints_vec[0]["head_pan_joint"] + pan_deltas[i];
      for (size_t j = 0; j < tilt_deltas.size(); j++){

        query_joint_map["head_tilt_joint"] = robot_joints_vec[0]["head_tilt_joint"] + tilt_deltas[j];
        
        if (query_joint_map["head_pan_joint"] < -3.0 || query_joint_map["head_pan_joint"] > 3.0 ||
            query_joint_map["head_tilt_joint"] < -1.2 || query_joint_map["head_tilt_joint"] > 0.1)
        {
          cost = -FLT_MAX;
        }
        else{
          query_joint_map["head_pan_joint"] = std::min(std::max(-3.0, (double) query_joint_map["head_pan_joint"] ), 3.0);
          query_joint_map["head_tilt_joint"] = std::min(std::max(-1.20, (double) query_joint_map["head_tilt_joint"] ), 0.1);
          timing::Timer single_view_cost_timer("single_view_cost_timer");
          cost = this->GetConeViewCost(query_joint_map);
          single_view_cost_timer.Stop();
        }

        costs.push_back(cost);
        // std::cout << cost << " | ";

      } 
      // std::cout << std::endl;
      // std::cout << "---------------------"<< std::endl;
    }

    int min_cost_idx = std::max_element( costs.begin(), costs.end()) - costs.begin();
    size_t pan_ind = floor((float) min_cost_idx / (float) tilt_deltas.size());
    size_t tilt_ind = min_cost_idx % tilt_deltas.size();
    
    // float pan_clamped = std::max(-0.7f, (float) std::min(pan_deltas[pan_ind], 0.7f));
    // pan_clamped += robot_joints_vec[0]["head_pan_joint"];
    // float tilt_clamped = std::max(-0.7f, (float) std::min(tilt_deltas[pan_ind], 0.7f));
    // tilt_clamped += robot_joints_vec[0]["head_tilt_joint"];
    // Clamp to the max deviation we want per timestep
    // Clamp to the absolute max and min of the joints (or what we set it to at least)
    // nbv_joints[0] =  std::min(std::max(-3.5f, pan_clamped), 1.6f);
    // nbv_joints[1] =  std::min(std::max(-1.20f, tilt_clamped), 0.1f);

    // Clamp to the absolute max and min of the joints (or what we set it to at least)
    nbv_joints[0] =  std::min(std::max(-3.0, (double) (robot_joints_vec[0]["head_pan_joint"] + pan_deltas[pan_ind])), 3.0);
    nbv_joints[1] =  std::min(std::max(-1.20, (double) (robot_joints_vec[0]["head_tilt_joint"] + tilt_deltas[tilt_ind])), 0.1);
    nbv_timer.Stop();
    


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