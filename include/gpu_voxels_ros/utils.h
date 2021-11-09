#ifndef GPU_VOXELS_ROS_UTILS_H
#define GPU_VOXELS_ROS_UTILS_H

#include <math.h>       /* fmod */

namespace gpu_voxels_ros {

  // Constrain angles between -pi and pi
  float constrainAngle(float x);

}  // namespace gpu_voxels_ros

#endif  // GPU_VOXELS_ROS_UTILS_H
