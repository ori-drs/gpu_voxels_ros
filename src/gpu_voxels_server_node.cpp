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

  // ros::spin();

  auto begin = std::chrono::system_clock::now();
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed;

  while(true)
  {
    ros::spinOnce();
  
    end = std::chrono::system_clock::now();
    
    elapsed = end - begin;
    // std::cout << elapsed.count() - 60 << std::endl;

    if(elapsed.count() - 60 >= 0){
      break;
    }

  }
  return 0;
}
