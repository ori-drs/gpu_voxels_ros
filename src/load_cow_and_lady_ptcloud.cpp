#include <cstdlib>
#include <signal.h>

#include <gpu_voxels/GpuVoxels.h>
#include <gpu_voxels/helpers/MetaPointCloud.h>
#include <gpu_voxels/logging/logging_gpu_voxels.h>

#include <gpu_voxels/voxel/DistanceVoxel.h>
#include <gpu_voxels/voxelmap/DistanceVoxelMap.h>

#include <gpu_voxels/voxel/CountingVoxel.h>
#include <gpu_voxels/voxellist/CountingVoxelList.h>

#include <gpu_voxels/helpers/common_defines.h>
#include <gpu_voxels/helpers/PointcloudFileHandler.h>

#include <thrust/remove.h>
#include <string>
#include <sstream>
#include <fstream>

using boost::dynamic_pointer_cast;
using boost::shared_ptr;
using gpu_voxels::voxelmap::DistanceVoxelMap;
using gpu_voxels::voxellist::CountingVoxelList;

shared_ptr<GpuVoxels> gvl;
float voxel_side_length = 0.05f; 
const int  NXY = 400;
const int  NZ = 80;

template <uint32_t nxy, uint32_t nz>
struct out_of_bounds {
  bool operator()(Vector3f point) const {
    Vector3ui dimensions(nxy, nxy, nz);

    const Vector3ui int_coords = voxelmap::mapToVoxels(voxel_side_length, point);
//    const Vector3f int_coords = point; //TODO change

    //check if point is in the range of the voxel map
    if ((int_coords.x < dimensions.x) && (int_coords.y < dimensions.y)
        && (int_coords.z < dimensions.z))
    {
      return false;
    } else {
      return true;
    }
  }
};

int main(int argc, char* argv[])
{
    icl_core::logging::initialize(argc, argv);

    LOGGING_INFO(Gpu_voxels, "DistanceVoxelTest start" << endl);

    Vector3ui map_dimensions = Vector3ui(NXY, NXY, NZ);

    const int num_voxels = NXY * NXY * NZ;

    gvl = gpu_voxels::GpuVoxels::getInstance();
    gvl->initialize(NXY, NXY, NZ, voxel_side_length);

    // load obstacles
    LOGGING_INFO(Gpu_voxels, "loading obstacles:" << endl);

    std::vector<Vector3f> obstacles;
    Vector3f offset(0.0f);
    // Vector3f offset(5.0f, 3.0f, 3.0f);
    float scaling = 1.f;
    std::string pointcloud_filename("/home/mark/Documents/gpu_voxels_analysis/heatmaps/cow_and_lady_extras/cow_and_lady_gt.pcd");

    obstacles.clear();

    if(file_handling::PointcloudFileHandler::Instance()->loadPointCloud(pointcloud_filename, false, obstacles, true, offset, scaling)) {
        LOGGING_INFO(Gpu_voxels, "loading pointcloud from file "<< pointcloud_filename <<" succeeded!" << endl);
    } else {
        LOGGING_INFO(Gpu_voxels, "using fallback obstacles instead of pointcloud!" << endl);
    }

    LOGGING_INFO(Gpu_voxels, "obstacle count before filtering: "<<obstacles.size()<< endl);

    std::vector<Vector3f>::iterator new_end = thrust::remove_if(obstacles.begin(), obstacles.end(), out_of_bounds<NXY, NZ>());
    obstacles.erase(new_end, obstacles.end());
    //    obstacles.resize(new_end - obstacles.begin());

    LOGGING_INFO(Gpu_voxels, "obstacle count after bounds filtering: "<< obstacles.size() << endl);


    // //PBA
    // gvl->addMap(MT_DISTANCE_VOXELMAP, "pbaDistanceVoxmap");
    // shared_ptr<DistanceVoxelMap> pbaDistanceVoxmap = dynamic_pointer_cast<DistanceVoxelMap>(gvl->getMap("pbaDistanceVoxmap"));
    // pbaDistanceVoxmap->clearMap();
    // pbaDistanceVoxmap->insertPointCloud(obstacles, eBVM_OCCUPIED);

    //PBA
    gvl->addMap(MT_COUNTING_VOXELLIST, "countingVoxelList");
    shared_ptr<CountingVoxelList> countingVoxelList = dynamic_pointer_cast<CountingVoxelList>(gvl->getMap("countingVoxelList"));
    countingVoxelList->clearMap();
    countingVoxelList->insertPointCloud(obstacles, eBVM_OCCUPIED);

    float col_threshold = 0.8;
    std::vector<bool> occupancy_map;
    occupancy_map = std::vector<bool>(num_voxels);
    countingVoxelList->getOccupancyToHost(occupancy_map, col_threshold);

    // Save the GPU-Voxels Occupancy to file
    std::ofstream savefile;
    savefile.open("/home/mark/Documents/gpu_voxels_analysis/heatmaps/gpu_voxels_cow_and_lady_occupancy.txt", std::fstream::out);

    for (size_t i = 0; i < occupancy_map.size(); i++)
    {
        savefile << occupancy_map[i];  

        if (i>0 && ((i+1) % (NXY*NXY) == 0))
        {
            savefile << '\n';  
        }
        else
        {
            savefile << ',';  
        }
        
        // }
    }
    savefile.close();
    LOGGING_INFO(Gpu_voxels, "File saved successfully" << endl);


}