#include <ros/ros.h>
#include <gpu_voxels_ros/live_composite_sdf.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "gpu_voxels");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  gpu_voxels_ros::LiveCompositeSDF* gpu_voxels_ptr; 
  gpu_voxels_ptr = new gpu_voxels_ros::LiveCompositeSDF(nh);


  auto begin = std::chrono::system_clock::now();
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed;

  while(true)
  {
    ros::spinOnce();
  
    end = std::chrono::system_clock::now();
    
    elapsed = end - begin;
    // std::cout << elapsed.count() - 60 << std::endl;

    if(elapsed.count() - 240 >= 0){
      break;
    }

  }
  std::cout << "Exiting live composite node." << std::endl;
  return 0;
}
