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

#ifndef GPU_VOXELS_ROS_LIVE_COMPOSITE_SDF_H
#define GPU_VOXELS_ROS_LIVE_COMPOSITE_SDF_H

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


#include <geometry_msgs/PoseArray.h>
// #include <finean_msgs/HumanTrajectoryPrediction.h>
#include <std_msgs/Float32MultiArray.h>

#include <gpu_voxels_ros/utils.h>

using boost::dynamic_pointer_cast;

using gpu_voxels::voxelmap::ProbVoxelMap;
using gpu_voxels::voxelmap::DistanceVoxelMap;
using gpu_voxels::voxellist::CountingVoxelList;
using gpu_voxels::voxelmap::SignedDistanceVoxelMap;
using gpu_voxels::voxelmap::InheritSignedDistanceVoxelMap;

namespace gpu_voxels_ros{

  class LiveCompositeSDF {

    public:
      LiveCompositeSDF(ros::NodeHandle& node);
      ~LiveCompositeSDF();

      void PoseCallback(const geometry_msgs::TransformStampedConstPtr &msg);
      void PointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
      // void HumanTrajectoryPredictionCallback(const finean_msgs::HumanTrajectoryPrediction::ConstPtr& msg);
      void HumanTrajectoryPredictionCallback(const geometry_msgs::PoseArray::ConstPtr& msg);

      double QueryDistance(uint32_t xi, uint32_t yi, uint32_t zi) const;
      double GetTrilinearDistance(const Eigen::Vector3d &pos) const;
      double GetTrilinearDistanceAndGradient(const Eigen::Vector3d &pos, Eigen::Vector3d &grad) const;

      double QueryDistanceIndexed(uint32_t xi, uint32_t yi, uint32_t zi, size_t t_index) const;
      double GetDistanceAndGradientIndexed(const Eigen::Vector3d &pos, Eigen::Vector3d &grad, size_t t_index) const;
      double GetTrilinearDistanceAndGradientIndexed(const Eigen::Vector3d &pos, Eigen::Vector3d &grad, size_t t_index) const;
      double GetTrilinearDistanceIndexed(const Eigen::Vector3d &pos, size_t t_index) const;

      double GetDistanceAndGradient(const Eigen::Vector3d &pos, Eigen::Vector3d &grad) const;
      double GetDistance(const Eigen::Vector3d &pos) const;
      void CallbackSync();
      void SaveSDFToFile(const std::string filepath);
      void SaveOccupancyToFile(const std::string filepath);

      void publish2DDistanceFieldImage(const std::vector<float> &distance_field_2d);
      void publish2DDistanceField(const std::vector<float> &distance_field_2d);
      void publishRVIZOccupancy(const std::vector<int> &occupancy_map);
      void publishRVIZOccupancy(const std::vector<float> &sdf_map);
      void publishRVIZOccupancy(const std::vector<gpu_voxels::VectorSdfGrad> &sdf_grad_map);
      void publishRVIZGroundSDF(const std::vector<gpu_voxels::VectorSdfGrad> &sdf_grad_map);
      void publishRVIZGroundSDF(const std::vector<float> &sdf_map);
      void publishRVIZGroundSDFGrad(const std::vector<gpu_voxels::VectorSdfGrad> &sdf_grad_map);
      void publishRVIZTrajSweepOccupancy(const std::vector<int> &occupancy_map);
      void publishRVIZCostmap(const std::vector<float> &costmap);

      // NBV
      void SetConeFlags(robot::JointValueMap robot_joints);
      void GetNBV(std::vector<robot::JointValueMap> robot_joints_vec, float (&nbv_joints)[2], const size_t current_ind);
      float GetConeViewCost(robot::JointValueMap robot_joints);
      void publishRVIZUpdateTimes(const std::vector<uint16_t> &time_map, uint16_t threshold);
      void publishRVIZVoxelFlags(const std::vector<bool> &flag_map);

      float getPercentageMapExplored() const;

    private:
      ros::NodeHandle node_;
      std::string transform_topic_, pcl_topic_, sensor_frame_, traj_pred_topic_;
      ros::Subscriber pcl_sub_, transform_sub_, traj_pred_sub_;  
      ros::Publisher map_pub_, ground_sdf_pub_, ground_sdf_grad_pub_, update_time_pub_, cone_flag_pub_, traj_sweep_pub_, costmap_pub_, distance_field_2d_pub_, ground2dsdf_pub_;

      boost::shared_ptr<GpuVoxels> gvl_;
      boost::shared_ptr<DistanceVoxelMap> pbaDistanceVoxmap_, pbaInverseDistanceVoxmap_, pbaDistanceVoxmapVisual_;
      boost::shared_ptr<InheritSignedDistanceVoxelMap> signedDistanceMap_;
      boost::shared_ptr<DistanceVoxelMap> distvoxelmap_2d_;

      // boost::shared_ptr<ProbVoxelMap> erodeTempVoxmap1_, erodeTempVoxmap2_, maintainedProbVoxmap_, robotVoxmap_, cleanVoxmap_;
      boost::shared_ptr<ProbVoxelMap> maintainedProbVoxmap_, robotVoxmap_, cleanVoxmap_, cleanVoxmapVisual_;
      // boost::shared_ptr<CountingVoxelList> countingVoxelList_, countingVoxelListFiltered_;

      boost::shared_ptr<gpu_voxels::robot::UrdfRobot> robot_ptr_;

      Vector3ui human_dims_;
      Vector3ui cylinder_base_corner_;
      boost::shared_ptr<DistanceVoxelMap> human_shared_ptr_;
      std::vector<std::shared_ptr<std::vector<float>>> composite_sdf_ptrs_;
      std::vector<std::shared_ptr<std::vector<gpu_voxels::VectorSdfGrad>>> composite_sdf_grad_ptrs_;
      size_t num_sdfs_;

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
      
      size_t ind_delay_ = 2;

      // These are the camera specific field of view parametera.
      float dalpha_ = 1.1*M_PI_4;
      float dtheta_ = 1.3*M_PI_4;


      float max_ray_length_ = 7; // Testing whether this helps clearing


      Matrix4f tf_;
      Matrix4f sync_tf_;
      Matrix4f T_B_C_, T_D_B_;

      std::queue<std::tuple<ros::Time, Matrix4f>> cam_transform_queue_;
      std::queue<sensor_msgs::PointCloud2::ConstPtr> pointcloud_queue_;
      // finean_msgs::HumanTrajectoryPrediction::ConstPtr human_traj_latest_;
      geometry_msgs::PoseArray::ConstPtr human_traj_latest_;

      std::vector<gpu_voxels::VectorSdfGrad> sdf_grad_map_;
      std::vector<float> sdf_map_;
      std::vector<uint> time_update_map_;
      std::vector<bool> flag_map_;
      std::vector<int> occupancy_map_;
      std::vector<int> traj_step_map_;
      std::vector<float> host_costmap_;
      std::vector<float> host_2d_dist_;

  };
} // namespace

#endif //GPU_VOXELS_ROS_LIVE_COMPOSITE_SDF_H