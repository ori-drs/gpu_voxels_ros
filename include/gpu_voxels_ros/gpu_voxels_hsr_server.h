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

#ifndef GPU_VOXELS_ROS_HSR_SERVER_H
#define GPU_VOXELS_ROS_HSR_SERVER_H

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

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

using boost::dynamic_pointer_cast;

using gpu_voxels::voxelmap::ProbVoxelMap;
using gpu_voxels::voxelmap::DistanceVoxelMap;
using gpu_voxels::voxellist::CountingVoxelList;
using gpu_voxels::voxelmap::SignedDistanceVoxelMap;
using gpu_voxels::voxelmap::InheritSignedDistanceVoxelMap;

namespace gpu_voxels_ros{

  class GPUVoxelsHSRServer {

    public:
      GPUVoxelsHSRServer(ros::NodeHandle& node);
      ~GPUVoxelsHSRServer();

      void PoseCallback(const geometry_msgs::TransformStampedConstPtr &msg);
      void PointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

      double QueryDistance(uint32_t xi, uint32_t yi, uint32_t zi) const;
      double GetTrilinearDistance(const Eigen::Vector3d &pos) const;
      double GetTrilinearDistanceAndGradient(const Eigen::Vector3d &pos, Eigen::Vector3d &grad) const;

      double GetDistanceAndGradient(const Eigen::Vector3d &pos, Eigen::Vector3d &grad) const;
      double GetDistance(const Eigen::Vector3d &pos) const;
      void CallbackSync();
      void SaveSDFToFile(const std::string filepath);
      void SaveOccupancyToFile(const std::string filepath);

      void publishRVIZUpdateTimes(const std::vector<uint16_t> &time_map, uint16_t threshold);
      void publishRVIZVoxelFlags(const std::vector<bool> &flag_map);

      void publishRVIZOccupancy(const std::vector<int> &occupancy_map);
      void publishRVIZOccupancy(const std::vector<float> &sdf_map);
      void publishRVIZOccupancy(const std::vector<gpu_voxels::VectorSdfGrad> &sdf_grad_map);
      void publishRVIZGroundSDF(const std::vector<gpu_voxels::VectorSdfGrad> &sdf_grad_map);
      void publishRVIZGroundSDF(const std::vector<float> &sdf_map);
      void publishRVIZGroundSDFGrad(const std::vector<gpu_voxels::VectorSdfGrad> &sdf_grad_map);

      void GetNBV(std::vector<robot::JointValueMap> robot_joints_vec, float (&nbv_joints)[2]);
      float GetConeViewCost(robot::JointValueMap robot_joints);

    private:
      ros::NodeHandle node_;
      std::string transform_topic_, pcl_topic_, sensor_frame_;
      ros::Subscriber pcl_sub_, transform_sub_;  
      ros::Publisher map_pub_, ground_sdf_pub_, ground_sdf_grad_pub_, update_time_pub_, cone_flag_pub_;

      boost::shared_ptr<GpuVoxels> gvl_;
      boost::shared_ptr<DistanceVoxelMap> pbaDistanceVoxmap_, pbaInverseDistanceVoxmap_, pbaDistanceVoxmapVisual_;
      boost::shared_ptr<InheritSignedDistanceVoxelMap> signedDistanceMap_;

      boost::shared_ptr<ProbVoxelMap> erodeTempVoxmap1_, erodeTempVoxmap2_, maintainedProbVoxmap_, robotVoxmap_;
      boost::shared_ptr<CountingVoxelList> countingVoxelList_, countingVoxelListFiltered_;

      boost::shared_ptr<gpu_voxels::robot::UrdfRobot> robot_ptr_;

      float voxel_side_length_; 
      bool new_data_received_;
      Vector3ui map_dimensions_;
      PointCloud my_point_cloud_;
      bool remove_floor_;

      // Kinect v1 (Cow and Lady)
      // float min_ray_length_ = 0.4;
      // float max_ray_length_ = 4.0;

      pcl::PointCloud<pcl::PointXYZ> cloud_;

      // ASUS Xtion Pro Live (HSR)
      float min_ray_length_ = 0.8;
      // float max_ray_length_ = 3.5;



      float max_ray_length_ = 7; // Testing whether this helps clearing


      Matrix4f tf_;
      Matrix4f sync_tf_;
      Matrix4f T_B_C_, T_D_B_;

      std::queue<std::tuple<ros::Time, Matrix4f>> cam_transform_queue_;
      std::queue<sensor_msgs::PointCloud2::ConstPtr> pointcloud_queue_;

      std::vector<gpu_voxels::VectorSdfGrad> sdf_grad_map_;
      std::vector<float> sdf_map_;
      std::vector<uint16_t> time_update_map_;
      std::vector<bool> flag_map_;
      std::vector<int> occupancy_map_;
 
  };
} // namespace

#endif