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
#include <gpu_voxels_tester/gpu_voxels_server.h>

namespace gpu_voxels_tester{

  GPUVoxelsServer::GPUVoxelsServer(ros::NodeHandle& node) {

    // std::string point_cloud_topic = "/hsrb/head_rgbd_sensor/depth_registered/points";
    pcl_sub_ = node.subscribe<pcl::PointCloud<pcl::PointXYZ> >("/hsrb/head_rgbd_sensor/depth_registered/points", 1, &GPUVoxelsServer::PointcloudCallback, this);
    transform_sub_ = node.subscribe("/camera_pose", 10, &GPUVoxelsServer::PoseCallback, this);

    // Generate a GPU-Voxels instance:
    gvl_ = gpu_voxels::GpuVoxels::getInstance();
    gvl_->initialize(map_dimensions_.x, map_dimensions_.y, map_dimensions_.z, voxel_side_length_);

    gvl_->addMap(MT_DISTANCE_VOXELMAP, "pbaDistanceVoxmap");
    gvl_->addMap(MT_DISTANCE_VOXELMAP, "pbaInverseDistanceVoxmap");
    gvl_->addMap(MT_PROBAB_VOXELMAP, "erodeTempVoxmap1");
    gvl_->addMap(MT_PROBAB_VOXELMAP, "erodeTempVoxmap2");
    gvl_->addMap(MT_COUNTING_VOXELLIST, "countingVoxelList");
    gvl_->addMap(MT_COUNTING_VOXELLIST, "countingVoxelListFiltered");
    gvl_->addMap(MT_DISTANCE_VOXELMAP, "pbaDistanceVoxmapVisual");

    pbaDistanceVoxmap_ = dynamic_pointer_cast<DistanceVoxelMap>(gvl_->getMap("pbaDistanceVoxmap"));
    pbaInverseDistanceVoxmap_ = dynamic_pointer_cast<DistanceVoxelMap>(gvl_->getMap("pbaInverseDistanceVoxmap"));
    erodeTempVoxmap1_ = dynamic_pointer_cast<ProbVoxelMap>(gvl_->getMap("erodeTempVoxmap1"));
    erodeTempVoxmap2_ = dynamic_pointer_cast<ProbVoxelMap>(gvl_->getMap("erodeTempVoxmap2"));
    countingVoxelList_ = dynamic_pointer_cast<CountingVoxelList>(gvl_->getMap("countingVoxelList"));
    countingVoxelListFiltered_ = dynamic_pointer_cast<CountingVoxelList>(gvl_->getMap("countingVoxelListFiltered"));
    pbaDistanceVoxmapVisual_ = dynamic_pointer_cast<DistanceVoxelMap>(gvl_->getMap("pbaDistanceVoxmapVisual"));

    sdf_grad_map_ = std::vector<gpu_voxels::VectorSdfGrad>(pbaDistanceVoxmap_->getVoxelMapSize());

    new_data_received_ = true; // call visualize on the first iteration
  }

  void GPUVoxelsServer::PointcloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
  {
    std::cout << "PointcloudCallback..." << std::endl;

    std::vector<Vector3f> point_data;
    point_data.resize(msg->points.size());

    for (uint32_t i = 0; i < msg->points.size(); i++)
    {
      point_data[i].x = msg->points[i].x;
      point_data[i].y = msg->points[i].y;
      point_data[i].z = msg->points[i].z;
    }

    //my_point_cloud.add(point_data);
    my_point_cloud_.update(point_data);
    
    // Vector3f pos(map_dimensions_.x * voxel_side_length_ * 0.5f, -0.2f, map_dimensions_.z * voxel_side_length_ * 0.5f); // camera located at y=-0.2m, x_max/2, z_max/2

    // float roll = 0.0;
    // float pitch = 0.0;
    // float yaw = 0.0;

    // tf_ = Matrix4f::createFromRotationAndTranslation(Matrix3f::createFromRPY(-3.14/2.0 + roll, 0 + pitch, 0 + yaw), pos);


    // transform new pointcloud to world coordinates
    my_point_cloud_.transformSelf(&tf_);
    
    new_data_received_ = true;

    // ------------------------------------------------------ Not in original

    pbaDistanceVoxmap_->clearMap();
    pbaInverseDistanceVoxmap_->clearMap();
    countingVoxelList_->clearMap();
    countingVoxelListFiltered_->clearMap();
    erodeTempVoxmap1_->clearMap();
    erodeTempVoxmap2_->clearMap();

    countingVoxelList_->insertPointCloud(my_point_cloud_, eBVM_OCCUPIED);
    countingVoxelListFiltered_->merge(countingVoxelList_);
    countingVoxelListFiltered_->remove_underpopulated(filter_threshold_);

    erodeTempVoxmap1_->merge(countingVoxelListFiltered_);
    if (erode_threshold_ > 0)
    {
      erodeTempVoxmap1_->erodeInto(*erodeTempVoxmap2_, erode_threshold_);
    } else
    {
      erodeTempVoxmap1_->erodeLonelyInto(*erodeTempVoxmap2_); //erode only "lonely voxels" without occupied neighbors
    }
    pbaDistanceVoxmap_->mergeOccupied(erodeTempVoxmap2_);
    pbaInverseDistanceVoxmap_->mergeFree(erodeTempVoxmap2_);

    // Calculate the distance map:
    pbaDistanceVoxmap_->parallelBanding3D();

    // Calculate the inverse distance map:
    pbaInverseDistanceVoxmap_->parallelBanding3D();

    // std::vector<gpu_voxels::VectorSdfGrad> sdf_grad_map(pbaDistanceVoxmap_->getVoxelMapSize());
    pbaDistanceVoxmap_->getSignedDistancesAndGradientsToHost(pbaInverseDistanceVoxmap_, sdf_grad_map_);

    pbaDistanceVoxmapVisual_->clone(*(pbaDistanceVoxmap_.get()));

    gvl_->visualizeMap("pbaDistanceVoxmap");
    std::cout << "New visual available..." << std::endl;

  }

  // TODO - Need to add in an offset to account for an origin and negative position values
  double GPUVoxelsServer::GetDistanceAndGradient(const Eigen::Vector3d &pos, Eigen::Vector3d &grad){
    
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

  double GPUVoxelsServer::GetDistance(const Eigen::Vector3d &pos){
    uint lin_ind = gpu_voxels::voxelmap::getVoxelIndexUnsigned(map_dimensions_, gpu_voxels::Vector3ui(pos[0], pos[1], pos[2]));
    
    return sdf_grad_map_[lin_ind].sdf;
  }

  void GPUVoxelsServer::PoseCallback(const geometry_msgs::TransformStampedConstPtr &msg) {

    std::cout << "PoseCallback..." << std::endl;

    // Vector3f pos(map_dimensions_.x * voxel_side_length_ * 0.5f, -0.2f, map_dimensions_.z * voxel_side_length_ * 0.5f); // camera located at y=-0.2m, x_max/2, z_max/2

    // float roll = 0.0;
    // float pitch = 0.0;
    // float yaw = 0.0;

    // Vector3f pos(msg->transform.translation.x,
    //             msg->transform.translation.y,
    //             msg->transform.translation.z);

    Vector3f pos(1.2f,
                1.2f,
                map_dimensions_.z * voxel_side_length_ * 0.5f);

    tf::Quaternion q(msg->transform.rotation.x,
                    msg->transform.rotation.y,
                    msg->transform.rotation.z,
                    msg->transform.rotation.w);

    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    std::cout << "roll: " << roll << "pitch: " << pitch << "yaw: " << yaw << std::endl;

    tf_ = Matrix4f::createFromRotationAndTranslation(Matrix3f::createFromRPY(0 + roll, 0 + pitch, 0 + yaw), pos);
    // tf_ = Matrix4f::createFromRotationAndTranslation(Matrix3f::createFromRPY(-3.14/2.0 + roll, 0 + pitch, 0 + yaw), pos);

  }


} // namespace 