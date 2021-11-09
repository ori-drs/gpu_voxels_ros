#include <gpu_voxels_ros/utils.h>

// Constrain angles between -pi and pi
float gpu_voxels_ros::constrainAngle(float x){
    x = fmod(x + M_PI, 2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}