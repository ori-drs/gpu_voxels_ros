#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "gpu_voxels");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

//   gpuvoxels::SomethingServer node(nh, nh_private);

  ros::spin();
  return 0;
}
