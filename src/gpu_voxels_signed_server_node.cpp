#include <ros/ros.h>
#include <gpu_voxels_ros/gpu_voxels_signed_server.h>
#include <chrono>

int main(int argc, char** argv) {
  ros::init(argc, argv, "gpu_voxels");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string savepath = "/home/mark/Documents/gpu_voxels_analysis/heatmaps/gpu_voxels_cow_and_lady_sdf_normalised2.txt";
  std::string occsavepath = "/home/mark/Documents/gpu_voxels_analysis/heatmaps/gpu_voxels_cow_and_lady_occupancy.txt";

  gpu_voxels_ros::GPUVoxelsSignedServer* gpu_voxels_ptr; 
  gpu_voxels_ptr = new gpu_voxels_ros::GPUVoxelsSignedServer(nh);

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

    if(elapsed.count() - 220 >= 0){
      gpu_voxels_ptr->SaveSDFToFile(savepath);
      gpu_voxels_ptr->SaveOccupancyToFile(occsavepath);
      break;
    }

  }

  return 0;
}