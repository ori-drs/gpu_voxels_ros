#include <ros/ros.h>
#include <gpu_voxels_ros/gpu_voxels_signed_server.h>
#include <chrono>

int main(int argc, char** argv) {
  ros::init(argc, argv, "gpu_voxels");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  gpu_voxels_ros::GPUVoxelsSignedServer* gpu_voxels_ptr; 
  gpu_voxels_ptr = new gpu_voxels_ros::GPUVoxelsSignedServer(nh);

  auto begin = std::chrono::system_clock::now();
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed;

  while(true)
  {
    ros::spinOnce();
  
    end = std::chrono::system_clock::now();
    
    elapsed = end - begin;

    if(elapsed.count() - 220 >= 0){
      break;
    }

  }

  return 0;
}