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
#include <gpu_voxels_ros/gpu_voxels_server.h>
#include <gpu_voxels_ros/timing.h>

namespace gpu_voxels_ros{

  GPUVoxelsServer::GPUVoxelsServer(ros::NodeHandle& node) {

    // Cow and lady camera calibration
    T_B_C_ = Matrix4f(1, 0, 0, 0,
                      0, 1, 0, 0,
                      0, 0, 1, 0,
                      0, 0, 0, 1);

    T_D_B_= Matrix4f(0.971048, -0.120915, 0.206023, 0.00114049,
                    0.15701, 0.973037, -0.168959, 0.0450936,
                    -0.180038, 0.196415, 0.96385, 0.0430765,
                    0.0, 0.0, 0.0, 1.0);

    // std::string point_cloud_topic = "/hsrb/head_rgbd_sensor/depth_registered/points";
    // transform_sub_ = node.subscribe("/camera_pose", 10, &GPUVoxelsServer::PoseCallback, this);
    // pcl_sub_ = node.subscribe("/hsrb/head_rgbd_sensor/depth_registered/points", 10, &GPUVoxelsServer::PointcloudCallback, this);

    transform_sub_ = node.subscribe("/kinect/vrpn_client/estimated_transform", 10, &GPUVoxelsServer::PoseCallback, this);
    pcl_sub_ = node.subscribe("/camera/depth_registered/points", 10, &GPUVoxelsServer::PointcloudCallback, this);

    // Generate a GPU-Voxels instance:
    gvl_ = gpu_voxels::GpuVoxels::getInstance();
    gvl_->initialize(map_dimensions_.x, map_dimensions_.y, map_dimensions_.z, voxel_side_length_);

    gvl_->addMap(MT_DISTANCE_VOXELMAP, "pbaDistanceVoxmap");
    gvl_->addMap(MT_DISTANCE_VOXELMAP, "pbaInverseDistanceVoxmap");
    // gvl_->addMap(MT_PROBAB_VOXELMAP, "erodeTempVoxmap1");
    // gvl_->addMap(MT_PROBAB_VOXELMAP, "erodeTempVoxmap2");
    gvl_->addMap(MT_PROBAB_VOXELMAP, "maintainedProbVoxmap");
    // gvl_->addMap(MT_COUNTING_VOXELLIST, "countingVoxelList");
    // gvl_->addMap(MT_COUNTING_VOXELLIST, "countingVoxelListFiltered");
    gvl_->addMap(MT_DISTANCE_VOXELMAP, "pbaDistanceVoxmapVisual");

    pbaDistanceVoxmap_ = dynamic_pointer_cast<DistanceVoxelMap>(gvl_->getMap("pbaDistanceVoxmap"));
    pbaInverseDistanceVoxmap_ = dynamic_pointer_cast<DistanceVoxelMap>(gvl_->getMap("pbaInverseDistanceVoxmap"));
    // erodeTempVoxmap1_ = dynamic_pointer_cast<ProbVoxelMap>(gvl_->getMap("erodeTempVoxmap1"));
    // erodeTempVoxmap2_ = dynamic_pointer_cast<ProbVoxelMap>(gvl_->getMap("erodeTempVoxmap2"));
    maintainedProbVoxmap_ = dynamic_pointer_cast<ProbVoxelMap>(gvl_->getMap("maintainedProbVoxmap"));
    // countingVoxelList_ = dynamic_pointer_cast<CountingVoxelList>(gvl_->getMap("countingVoxelList"));
    // countingVoxelListFiltered_ = dynamic_pointer_cast<CountingVoxelList>(gvl_->getMap("countingVoxelListFiltered"));
    pbaDistanceVoxmapVisual_ = dynamic_pointer_cast<DistanceVoxelMap>(gvl_->getMap("pbaDistanceVoxmapVisual"));

    sdf_grad_map_ = std::vector<gpu_voxels::VectorSdfGrad>(pbaDistanceVoxmap_->getVoxelMapSize());
    sdf_map_ = std::vector<float>(pbaDistanceVoxmap_->getVoxelMapSize());

    // new_data_received_ = true; // call visualize on the first iteration
    maintainedProbVoxmap_->clearMap();

  }

  void GPUVoxelsServer::CallbackSync(){

    // std::cout << "CallbackSync..." << std::endl;
    // auto start = std::chrono::high_resolution_clock::now(); 
    timing::Timer sync_callback_timer("CallbackSync");

    ros::Time pcl_time;
    double time_delay = 3e-3;

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

        // std::cout << "Pointcloud Size:\t" << cloud_.size() << std::endl;
        
        if ((int) cloud_.size()==0) {
              pointcloud_queue_.pop();
              continue;
        }

        Vector3f camera_pos = Vector3f(sync_tf_.a14, sync_tf_.a24, sync_tf_.a34)/sync_tf_.a44;

        update_transform_timer.Stop();

        timing::Timer transform_pc_timer("ResizePC");

        std::vector<Vector3f> point_data;
        point_data.resize(cloud_.points.size());

        for (uint32_t i = 0; i < cloud_.points.size(); i++)
        {
          point_data[i].x = cloud_.points[i].x;
          point_data[i].y = cloud_.points[i].y;
          point_data[i].z = cloud_.points[i].z;
        }


        my_point_cloud_.update(point_data);
        
        // transform new pointcloud to world coordinates
        // my_point_cloud_.transformSelf(&tf_);
        my_point_cloud_.transformSelf(&sync_tf_);
        
        transform_pc_timer.Stop();

        // new_data_received_ = true;

        // ------------------------------------------------------ Not in original


        timing::Timer update_esdf_timer("UpdateESDF");

        pbaDistanceVoxmap_->clearMap();
        pbaInverseDistanceVoxmap_->clearMap();
        // countingVoxelList_->clearMap();
        // countingVoxelListFiltered_->clearMap();
        // erodeTempVoxmap1_->clearMap();
        // erodeTempVoxmap2_->clearMap();

        // countingVoxelList_->insertPointCloud(my_point_cloud_, eBVM_OCCUPIED);
        // countingVoxelListFiltered_->merge(countingVoxelList_);

        // This removes voxels which don't have enough points corresponding to the voxel
        // countingVoxelListFiltered_->remove_underpopulated(filter_threshold_);

        // erodeTempVoxmap1_->merge(countingVoxelListFiltered_);
        // if (erode_threshold_ > 0)
        // {
        //   erodeTempVoxmap1_->erodeInto(*erodeTempVoxmap2_, erode_threshold_);
        // } else
        // {
        //   erodeTempVoxmap1_->erodeLonelyInto(*erodeTempVoxmap2_); //erode only "lonely voxels" without occupied neighbors
        // }
        
        maintainedProbVoxmap_->insertSensorData<BIT_VECTOR_LENGTH>(my_point_cloud_, camera_pos, true, false, eBVM_OCCUPIED, NULL);

        // This does a final check of occupancy threshold before inserting
        // pbaDistanceVoxmap_->mergeOccupied(erodeTempVoxmap2_, Vector3ui(), 0.8);
        // pbaInverseDistanceVoxmap_->mergeFree(erodeTempVoxmap2_, Vector3ui(), 0.8);
        pbaDistanceVoxmap_->mergeOccupied(maintainedProbVoxmap_, Vector3ui(), 0.8);
        pbaInverseDistanceVoxmap_->mergeFree(maintainedProbVoxmap_, Vector3ui(), 0.5);

        // Calculate the distance map:
        pbaDistanceVoxmap_->parallelBanding3D();
        // Calculate the inverse distance map:
        pbaInverseDistanceVoxmap_->parallelBanding3D();        
        
        update_esdf_timer.Stop();

        timing::Timer transfer_timer("HostRetrieval");



        // std::vector<gpu_voxels::VectorSdfGrad> sdf_grad_map(pbaDistanceVoxmap_->getVoxelMapSize());
        // pbaDistanceVoxmap_->getSignedDistancesAndGradientsToHost(pbaInverseDistanceVoxmap_, sdf_grad_map_);
        
        pbaDistanceVoxmap_->getSignedDistancesToHost(pbaInverseDistanceVoxmap_, sdf_map_);
        // pbaDistanceVoxmap_->getUnsignedDistancesToHost(sdf_map_);
        transfer_timer.Stop();
        sync_callback_timer.Stop();
        // pbaDistanceVoxmap_->getSignedDistancesAndGradientsToHost(pbaInverseDistanceVoxmap_, sdf_grad_map_);
        pointcloud_queue_.pop();

        // auto finish = std::chrono::high_resolution_clock::now(); 
        // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(finish - start); 
        // double dur = duration.count();
        // std::cout << "Callback time calculation time: " <<  dur << " microseconds." << std::endl;

        timing::Timing::Print(std::cout);

        pbaDistanceVoxmapVisual_->clone(*(pbaDistanceVoxmap_.get()));

        gvl_->visualizeMap("pbaDistanceVoxmapVisual");
        // std::cout << "New visual available..." << std::endl;
        

    }
  }

  void GPUVoxelsServer::PointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    pointcloud_queue_.push(msg);
    CallbackSync();
  }

  // TODO - Need to add in an offset to account for an origin and negative position values
  double GPUVoxelsServer::GetDistanceAndGradient(const Eigen::Vector3d &pos, Eigen::Vector3d &grad){
    
    // std::cout << "requesting GetDistanceAndGradient..." << std::endl;
    // std::cout << "input pos    x: " << pos[0] << " y: " << pos[1] << " z: " << pos[2] << std::endl;
    
    // Check that the point is int bounds

    gpu_voxels::Vector3ui coords = gpu_voxels::voxelmap::mapToVoxels(voxel_side_length_, gpu_voxels::Vector3f(pos[0], pos[1], pos[2]));
    uint lin_ind = gpu_voxels::voxelmap::getVoxelIndexUnsigned(map_dimensions_, coords);
    // std::cout << "accessing index: " << lin_ind << std::endl;

    grad[0] = sdf_grad_map_[lin_ind].x;
    grad[1] = sdf_grad_map_[lin_ind].y;
    grad[2] = sdf_grad_map_[lin_ind].y;

    return sdf_grad_map_[lin_ind].sdf;
  }

  double GPUVoxelsServer::GetDistance(const Eigen::Vector3d &pos){
    uint lin_ind = gpu_voxels::voxelmap::getVoxelIndexUnsigned(map_dimensions_, gpu_voxels::Vector3ui(pos[0], pos[1], pos[2]));
    
    return sdf_grad_map_[lin_ind].sdf;
  }

  void GPUVoxelsServer::PoseCallback(const geometry_msgs::TransformStampedConstPtr &msg) {

    // std::cout << "PoseCallback..." << std::endl;

    // Vector3f pos(map_dimensions_.x * voxel_side_length_ * 0.5f, -0.2f, map_dimensions_.z * voxel_side_length_ * 0.5f); // camera located at y=-0.2m, x_max/2, z_max/2

    // float roll = 0.0;
    // float pitch = 0.0;
    // float yaw = 0.0;

    Vector3f camera_pos = Vector3f(msg->transform.translation.x,
                            msg->transform.translation.y ,
                            msg->transform.translation.z);

    // camera_pos_ = Vector3f(1.2f,
                // 1.2f,
                // map_dimensions_.z * voxel_side_length_ * 0.5f);

    tf::Quaternion q(msg->transform.rotation.x,
                    msg->transform.rotation.y,
                    msg->transform.rotation.z,
                    msg->transform.rotation.w);

    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    // std::cout << "roll: " << roll << "pitch: " << pitch << "yaw: " << yaw << std::endl;

    tf_ = Matrix4f::createFromRotationAndTranslation(Matrix3f::createFromRPY(0 + roll, 0 + pitch, 0 + yaw), camera_pos);
    tf_ = tf_*T_D_B_*T_B_C_;
    tf_.a14 = tf_.a14  + map_dimensions_.x * voxel_side_length_ * 0.5f;
    tf_.a24 = tf_.a24 + map_dimensions_.y * voxel_side_length_ * 0.5f;
    // camera_pos_ = Vector3f(tf_.a14, tf_.a24, tf_.a34);


    // tf_ = Matrix4f::createFromRotationAndTranslation(Matrix3f::createFromRPY(-3.14/2.0 + roll, 0 + pitch, 0 + yaw), pos);
    
    cam_transform_queue_.push(std::make_tuple(msg->header.stamp, tf_));

  }


} // namespace 