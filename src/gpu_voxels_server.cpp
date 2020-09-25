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

  GPUVoxelsServer::GPUVoxelsServer(ros::NodeHandle node) {

    std::string point_cloud_topic = "/hsrb/head_rgbd_sensor/depth_registered/points";
    ros::Subscriber pcl_sub_ = node.subscribe<pcl::PointCloud<pcl::PointXYZ> >(point_cloud_topic, 1, &GPUVoxelsServer::pointcloud_callback, this);

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

    new_data_received_ = true; // call visualize on the first iteration
  }


  void GPUVoxelsServer::pointcloud_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
  {
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

    std::vector<float> sdf_map(pbaDistanceVoxmap_->getVoxelMapSize());
    pbaDistanceVoxmap_->getSignedDistancesToHost(pbaInverseDistanceVoxmap_, sdf_map);

    pbaDistanceVoxmapVisual_->clone(*(pbaInverseDistanceVoxmap_.get()));

    gvl_->visualizeMap("pbaInverseDistanceVoxmap");

  }
} // namespace 