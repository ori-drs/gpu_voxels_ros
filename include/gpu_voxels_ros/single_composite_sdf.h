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

#ifndef GPU_VOXELS_ROS_SINGLE_COMPOSITE_SDF_H
#define GPU_VOXELS_ROS_SINGLE_COMPOSITE_SDF_H

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
#include <std_msgs/Float32MultiArray.h>

#include <gpu_voxels_ros/utils.h>

#include <mutex> 

using boost::dynamic_pointer_cast;

using gpu_voxels::voxelmap::ProbVoxelMap;
using gpu_voxels::voxelmap::DistanceVoxelMap;
using gpu_voxels::voxellist::CountingVoxelList;
using gpu_voxels::voxelmap::SignedDistanceVoxelMap;
using gpu_voxels::voxelmap::InheritSignedDistanceVoxelMap;

namespace gpu_voxels_ros{

  class SingleCompositeSDF {

    public:
      SingleCompositeSDF(ros::NodeHandle& node);
      ~SingleCompositeSDF();

      void PoseCallback(const geometry_msgs::TransformStampedConstPtr &msg);
      void PointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

      void HumanTrajectoryPredictionCallback(const geometry_msgs::PoseArray::ConstPtr& msg);

      double QueryDistance(uint32_t xi, uint32_t yi, uint32_t zi) const;
      double GetTrilinearDistance(const Eigen::Vector3d &pos) const;
      double GetTrilinearDistanceAndGradient(const Eigen::Vector3d &pos, Eigen::Vector3d &grad) const;

      double GetDistanceAndGradient(const Eigen::Vector3d &pos, Eigen::Vector3d &grad) const;
      double GetDistance(const Eigen::Vector3d &pos) const;
      void CallbackSync();
      void SaveSDFToFile(const std::string filepath);
      void SaveOccupancyToFile(const std::string filepath);

      void publish2DDistanceFieldImage(const std::vector<float> &distance_field_2d);
      void publish2DDistanceField(const std::vector<float> &distance_field_2d);
      void publishRVIZOccupancy(const std::vector<int> &occupancy_map);
      void publishRVIZOccupancy(const std::vector<float> &sdf_map);
      void publishRVIZUncleanOccupancy(const std::vector<float> &sdf_map);
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
      ros::Publisher unclean_map_pub_, map_pub_, ground_sdf_pub_, ground_sdf_grad_pub_, update_time_pub_, cone_flag_pub_, traj_sweep_pub_, costmap_pub_, distance_field_2d_pub_, ground2dsdf_pub_;

      boost::shared_ptr<GpuVoxels> gvl_;
      boost::shared_ptr<DistanceVoxelMap> pbaDistanceVoxmap_, pbaInverseDistanceVoxmap_, pbaDistanceVoxmapVisual_;
      boost::shared_ptr<InheritSignedDistanceVoxelMap> signedDistanceMap_;
      boost::shared_ptr<SignedDistanceVoxelMap> humanSignedDistanceMap_;
      boost::shared_ptr<DistanceVoxelMap> distvoxelmap_2d_;

      // boost::shared_ptr<ProbVoxelMap> erodeTempVoxmap1_, erodeTempVoxmap2_, maintainedProbVoxmap_, robotVoxmap_, cleanVoxmap_;
      boost::shared_ptr<ProbVoxelMap> maintainedProbVoxmap_, robotVoxmap_, cleanVoxmap_, cleanVoxmapVisual_;
      // boost::shared_ptr<CountingVoxelList> countingVoxelList_, countingVoxelListFiltered_;

      boost::shared_ptr<gpu_voxels::robot::UrdfRobot> robot_ptr_;

      Vector3ui human_dims_;
      Vector3ui cylinder_base_corner_;
      boost::shared_ptr<DistanceVoxelMap> human_shared_ptr_, human_inverse_shared_ptr_;
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


      float max_ray_length_ = 15; // Testing whether this helps clearing


      Matrix4f tf_;
      Matrix4f sync_tf_;
      Matrix4f T_B_C_, T_D_B_;

      std::queue<std::tuple<ros::Time, Matrix4f>> cam_transform_queue_;
      std::queue<sensor_msgs::PointCloud2::ConstPtr> pointcloud_queue_;
      geometry_msgs::PoseArray::ConstPtr human_traj_latest_;
      std::mutex traj_msg_mutex_;

      std::vector<gpu_voxels::VectorSdfGrad> sdf_grad_map_;
      std::vector<float> sdf_map_;
      std::vector<uint> time_update_map_;
      std::vector<bool> flag_map_;
      std::vector<int> occupancy_map_;
      std::vector<int> traj_step_map_;
      std::vector<float> host_costmap_;
      std::vector<float> host_2d_dist_;
      std::vector<float> unclean_sdf_map_;

  };
} // namespace

#endif //GPU_VOXELS_ROS_LIVE_COMPOSITE_SDF_H