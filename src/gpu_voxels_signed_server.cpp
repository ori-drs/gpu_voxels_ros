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
#include <gpu_voxels_ros/gpu_voxels_signed_server.h>
#include <gpu_voxels_ros/timing.h>

namespace gpu_voxels_ros{

  GPUVoxelsSignedServer::GPUVoxelsSignedServer(ros::NodeHandle& node) {

    // Cow and lady camera calibration
    T_B_C_ = Matrix4f(1, 0, 0, 0,
                      0, 1, 0, 0,
                      0, 0, 1, 0,
                      0, 0, 0, 1);

    T_D_B_= Matrix4f(0.971048, -0.120915, 0.206023, 0.00114049,
                    0.15701, 0.973037, -0.168959, 0.0450936,
                    -0.180038, 0.196415, 0.96385, 0.0430765,
                    0.0, 0.0, 0.0, 1.0);

    transform_sub_ = node.subscribe("/kinect/vrpn_client/estimated_transform", 10, &GPUVoxelsSignedServer::PoseCallback, this);
    pcl_sub_ = node.subscribe("/camera/depth_registered/points", 10, &GPUVoxelsSignedServer::PointcloudCallback, this);

    // Generate a GPU-Voxels instance:
    gvl_ = gpu_voxels::GpuVoxels::getInstance();
    gvl_->initialize(map_dimensions_.x, map_dimensions_.y, map_dimensions_.z, voxel_side_length_);

    // gvl_->addMap(MT_DISTANCE_VOXELMAP, "pbaDistanceVoxmap");
    // gvl_->addMap(MT_DISTANCE_VOXELMAP, "pbaInverseDistanceVoxmap");
      // signedDistanceMap_ = new SignedDistanceVoxelMap(pbaDistanceVoxmap_, pbaInverseDistanceVoxmap_, map_dimensions_, voxel_side_length_);

    gvl_->addMap(MT_SIGNED_DISTANCE_VOXELMAP, "pbaDistanceVoxmap");
    gvl_->addMap(MT_PROBAB_VOXELMAP, "maintainedProbVoxmap");
    gvl_->addMap(MT_DISTANCE_VOXELMAP, "pbaDistanceVoxmapVisual");

    pbaDistanceVoxmap_ = dynamic_pointer_cast<DistanceVoxelMap>(gvl_->getMap("pbaDistanceVoxmap"));
    pbaInverseDistanceVoxmap_ = dynamic_pointer_cast<DistanceVoxelMap>(gvl_->getMap("pbaDistanceVoxmapInverse"));
    maintainedProbVoxmap_ = dynamic_pointer_cast<ProbVoxelMap>(gvl_->getMap("maintainedProbVoxmap"));
    pbaDistanceVoxmapVisual_ = dynamic_pointer_cast<DistanceVoxelMap>(gvl_->getMap("pbaDistanceVoxmapVisual"));

    signedDistanceMap_ = dynamic_pointer_cast<InheritSignedDistanceVoxelMap>(pbaDistanceVoxmap_);

    sdf_grad_map_ = std::vector<gpu_voxels::VectorSdfGrad>(pbaDistanceVoxmap_->getVoxelMapSize());
    sdf_map_ = std::vector<float>(pbaDistanceVoxmap_->getVoxelMapSize());
    occupancy_map_ = std::vector<int>(pbaDistanceVoxmap_->getVoxelMapSize());

    // new_data_received_ = true; // call visualize on the first iteration
    maintainedProbVoxmap_->clearMap();

    // signedDistanceMap_ = new SignedDistanceVoxelMap(pbaDistanceVoxmap_, pbaInverseDistanceVoxmap_, map_dimensions_, voxel_side_length_);
  }

  void GPUVoxelsSignedServer::CallbackSync(){

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
        my_point_cloud_.transformSelf(&sync_tf_);
        
        transform_pc_timer.Stop();

        timing::Timer update_esdf_timer("UpdateESDF");

        signedDistanceMap_->clearMaps();

        // maintainedProbVoxmap_->insertSensorData<BIT_VECTOR_LENGTH>(my_point_cloud_, camera_pos, true, false, eBVM_OCCUPIED, NULL);
        maintainedProbVoxmap_->insertClippedSensorData<BIT_VECTOR_LENGTH>(my_point_cloud_, camera_pos, true, false, eBVM_OCCUPIED, 
                                                                          min_ray_length, max_ray_length, NULL);

        signedDistanceMap_->occupancyMerge(maintainedProbVoxmap_, 0.8, 0.79999);

        signedDistanceMap_->parallelBanding3DMark();

        update_esdf_timer.Stop();

        timing::Timer transfer_timer("HostRetrieval");

        // std::vector<gpu_voxels::VectorSdfGrad> sdf_grad_map(pbaDistanceVoxmap_->getVoxelMapSize());
        // signedDistanceMap_->getSignedDistancesAndGradientsToHost(sdf_grad_map_);
        // signedDistanceMap_->getSignedDistancesToHost(sdf_map_);

        pbaDistanceVoxmap_->getUnsignedDistancesToHost(sdf_map_);
        // pbaDistanceVoxmap_->getSignedDistancesToHost(pbaInverseDistanceVoxmap_, sdf_map_);

        transfer_timer.Stop();

        sync_callback_timer.Stop();

        pointcloud_queue_.pop();

        timing::Timing::Print(std::cout);
        
        // pbaDistanceVoxmapVisual_->clone(*(pbaDistanceVoxmap_.get()));

        // gvl_->visualizeMap("pbaDistanceVoxmapVisual");        

    }
  }

  void GPUVoxelsSignedServer::PointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    pointcloud_queue_.push(msg);
    CallbackSync();
  }

  // TODO - Need to add in an offset to account for an origin and negative position values
  double GPUVoxelsSignedServer::GetDistanceAndGradient(const Eigen::Vector3d &pos, Eigen::Vector3d &grad){
    
    // std::cout << "requesting GetDistanceAndGradient..." << std::endl;
    // std::cout << "input pos    x: " << pos[0] << " y: " << pos[1] << " z: " << pos[2] << std::endl;
    gpu_voxels::Vector3ui coords = gpu_voxels::voxelmap::mapToVoxels(voxel_side_length_, gpu_voxels::Vector3f(pos[0], pos[1], pos[2]));
    uint lin_ind = gpu_voxels::voxelmap::getVoxelIndexUnsigned(map_dimensions_, coords);
    // std::cout << "accessing index: " << lin_ind << std::endl;

    grad[0] = sdf_grad_map_[lin_ind].x;
    grad[1] = sdf_grad_map_[lin_ind].y;
    grad[2] = sdf_grad_map_[lin_ind].y;

    return sdf_grad_map_[lin_ind].sdf;
  }

  double GPUVoxelsSignedServer::GetDistance(const Eigen::Vector3d &pos){
    uint lin_ind = gpu_voxels::voxelmap::getVoxelIndexUnsigned(map_dimensions_, gpu_voxels::Vector3ui(pos[0], pos[1], pos[2]));
    
    return sdf_grad_map_[lin_ind].sdf;
  }

  void GPUVoxelsSignedServer::PoseCallback(const geometry_msgs::TransformStampedConstPtr &msg) {

    Vector3f camera_pos = Vector3f(msg->transform.translation.x,
                            msg->transform.translation.y ,
                            msg->transform.translation.z);

    tf::Quaternion q(msg->transform.rotation.x,
                    msg->transform.rotation.y,
                    msg->transform.rotation.z,
                    msg->transform.rotation.w);

    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    tf_ = Matrix4f::createFromRotationAndTranslation(Matrix3f::createFromRPY(0 + roll, 0 + pitch, 0 + yaw), camera_pos);
    tf_ = tf_*T_D_B_*T_B_C_;

    tf_.a14 = tf_.a14  + map_dimensions_.x * voxel_side_length_ * 0.5f;
    tf_.a24 = tf_.a24 + map_dimensions_.y * voxel_side_length_ * 0.5f;

    cam_transform_queue_.push(std::make_tuple(msg->header.stamp, tf_));

  }

  void GPUVoxelsSignedServer::SaveSDFToFile(const std::string filepath){
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

  void GPUVoxelsSignedServer::SaveOccupancyToFile(const std::string filepath){
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