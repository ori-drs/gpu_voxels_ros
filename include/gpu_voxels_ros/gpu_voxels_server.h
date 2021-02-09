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
#ifndef GPU_VOXELS_ROS_SERVER_H
#define GPU_VOXELS_ROS_SERVER_H

#include <cstdlib>
#include <signal.h>

#include <gpu_voxels/GpuVoxels.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <icl_core_config/Config.h>
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Quaternion.h"

#include <chrono>
#include <Eigen/Eigen>
#include <queue> 

using boost::dynamic_pointer_cast;
// using boost::shared_ptr;
using gpu_voxels::voxelmap::ProbVoxelMap;
using gpu_voxels::voxelmap::DistanceVoxelMap;
using gpu_voxels::voxellist::CountingVoxelList;

namespace gpu_voxels_ros{

  class GPUVoxelsServer {

    public:
      GPUVoxelsServer(ros::NodeHandle& node);
      ~GPUVoxelsServer();

      void PoseCallback(const geometry_msgs::TransformStampedConstPtr &msg);
      void PointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

      double GetDistanceAndGradient(const Eigen::Vector3d &pos, Eigen::Vector3d &grad);
      double GetDistance(const Eigen::Vector3d &pos);
      void CallbackSync();

    private:
      ros::Subscriber pcl_sub_, transform_sub_;
      boost::shared_ptr<GpuVoxels> gvl_;
      boost::shared_ptr<DistanceVoxelMap> pbaDistanceVoxmap_, pbaInverseDistanceVoxmap_, pbaDistanceVoxmapVisual_;
      boost::shared_ptr<ProbVoxelMap> erodeTempVoxmap1_, erodeTempVoxmap2_, maintainedProbVoxmap_;
      boost::shared_ptr<CountingVoxelList> countingVoxelList_, countingVoxelListFiltered_;

      // float voxel_side_length_ = 0.05f; // 1 cm voxel size
      float voxel_side_length_ = 0.4f; // 1 cm voxel size
      bool new_data_received_;
      // Vector3ui map_dimensions_ = Vector3ui(448, 448, 128);
      Vector3ui map_dimensions_ = Vector3ui(64, 64, 16);
      PointCloud my_point_cloud_;
      // Vector3f camera_pos_;

      pcl::PointCloud<pcl::PointXYZ> cloud_;

      Matrix4f tf_;
      Matrix4f sync_tf_;
      Matrix4f T_B_C_, T_D_B_;

      std::queue<std::tuple<ros::Time, Matrix4f>> cam_transform_queue_;
      std::queue<sensor_msgs::PointCloud2::ConstPtr> pointcloud_queue_;

      // int filter_threshold_ = 0;
      // float erode_threshold_ = 0.0f;

      std::vector<gpu_voxels::VectorSdfGrad> sdf_grad_map_;
      std::vector<float> sdf_map_;
 
  };
} // namespace

#endif