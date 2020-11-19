#include <ros/ros.h>
#include <gpu_voxels_ros/gpu_voxels_server.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "gpu_voxels");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // gpu_voxels_ros::GPUVoxelsServer node(nh, nh_private);
  // gpu_voxels_ros::GPUVoxelsServer node(nh);
  gpu_voxels_ros::GPUVoxelsServer* gpu_voxels_ptr; 
  gpu_voxels_ptr = new gpu_voxels_ros::GPUVoxelsServer(nh);

  ros::spin();
  return 0;
}
