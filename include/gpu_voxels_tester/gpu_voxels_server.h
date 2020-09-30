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

using boost::dynamic_pointer_cast;
// using boost::shared_ptr;
using gpu_voxels::voxelmap::ProbVoxelMap;
using gpu_voxels::voxelmap::DistanceVoxelMap;
using gpu_voxels::voxellist::CountingVoxelList;

namespace gpu_voxels_tester{

  class GPUVoxelsServer {

    public:
      GPUVoxelsServer(ros::NodeHandle& node);
      ~GPUVoxelsServer();

      void PoseCallback(const geometry_msgs::TransformStampedConstPtr &msg);
      void PointcloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg);
      // void PointcloudCallback(const sensor_msgs::PointCloud2ConstPtr & msg);
      double GetDistanceAndGradient(const Eigen::Vector3d &pos, Eigen::Vector3d &grad);
      double GetDistance(const Eigen::Vector3d &pos);

    private:
      ros::Subscriber pcl_sub_, transform_sub_;
      boost::shared_ptr<GpuVoxels> gvl_;
      boost::shared_ptr<DistanceVoxelMap> pbaDistanceVoxmap_, pbaInverseDistanceVoxmap_, pbaDistanceVoxmapVisual_;
      boost::shared_ptr<ProbVoxelMap> erodeTempVoxmap1_, erodeTempVoxmap2_;
      boost::shared_ptr<CountingVoxelList> countingVoxelList_, countingVoxelListFiltered_;

      float voxel_side_length_ = 0.01f; // 1 cm voxel size
      bool new_data_received_;
      Vector3ui map_dimensions_ = Vector3ui(256, 256, 256);
      // Vector3f camera_offsets_ = Vector3f(map_dimensions_.x * voxel_side_length_ * 0.5f, 
      //                                     -0.2f, 
      //                                     map_dimensions_.z * voxel_side_length_ * 0.5f); // camera located at y=-0.2m, x_max/2, z_max/2 
      // Vector3f camera_offsets_;

      PointCloud my_point_cloud_;


      // float roll_ = 0;
      // float pitch_ = 0;
      // float yaw_ = 0;
      Matrix4f tf_;

      // Matrix4f tf_ = Matrix4f::createFromRotationAndTranslation(Matrix3f::createFromRPY(-3.14/2.0 + roll_, 0 + pitch_, 0 + yaw_), camera_offsets_);

      int filter_threshold_ = 0;
      float erode_threshold_ = 0.0f;

      std::vector<gpu_voxels::VectorSdfGrad> sdf_grad_map_;

  };
} // namespace