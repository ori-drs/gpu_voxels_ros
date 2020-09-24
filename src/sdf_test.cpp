#include <cstdlib>
#include <signal.h>

#include <gpu_voxels/GpuVoxels.h>
#include <gpu_voxels/helpers/GeometryGeneration.h>
#include <fstream>
// #include <ros/ros.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl/point_types.h>
// #include <icl_core_config/Config.h>

// #include <chrono>
// #include <string>

using namespace gpu_voxels;
using boost::shared_ptr;

typedef boost::shared_ptr<voxelmap::DistanceVoxelMap> DistMapSharedPtr;
typedef boost::shared_ptr<voxelmap::ProbVoxelMap> ProxVoxelMapSharedPtr;
typedef boost::shared_ptr<voxellist::CountingVoxelList> CountingVoxelListSharedPtr;

int main(int argc, char* argv[])
{
    float side_length = 1.f;

    int dimX = 64;
    int dimY = dimX;
    int dimZ = 64;

    // Create a distance field
    CountingVoxelListSharedPtr counting_voxel_list = CountingVoxelListSharedPtr(new voxellist::CountingVoxelList(Vector3ui(dimX, dimY, dimZ), side_length, MT_COUNTING_VOXELLIST));
    ProxVoxelMapSharedPtr erode_map = ProxVoxelMapSharedPtr(new voxelmap::ProbVoxelMap(Vector3ui(dimX, dimY, dimZ), side_length, MT_PROBAB_VOXELMAP));
    DistMapSharedPtr pba_dist_map = DistMapSharedPtr(new voxelmap::DistanceVoxelMap(Vector3ui(dimX, dimY, dimZ), side_length, MT_DISTANCE_VOXELMAP));
    DistMapSharedPtr pba_inverse_dist_map = DistMapSharedPtr(new voxelmap::DistanceVoxelMap(Vector3ui(dimX, dimY, dimZ), side_length, MT_DISTANCE_VOXELMAP));

    // Add obstacles
    Vector3f corner_min(0.0, 0.0, 0.0);
    Vector3f corner_max(10.0, 10.0, 10.0);
    std::vector<Vector3f> obstacles = gpu_voxels::geometry_generation::createBoxOfPoints(corner_min, corner_max, side_length);

    // Clear all maps
    pba_dist_map->clearMap();
    pba_inverse_dist_map->clearMap();
    counting_voxel_list->clearMap();
    erode_map->clearMap();

    // Add obstacles
    counting_voxel_list->insertPointCloud(obstacles, eBVM_OCCUPIED);
    erode_map->merge(counting_voxel_list);
    pba_dist_map->mergeOccupied(erode_map);
    pba_inverse_dist_map->mergeFree(erode_map);

    // Calculate distance transforms
    std::vector<gpu_voxels::VectorSdfGrad> sdf_grad_map(pba_dist_map->getVoxelMapSize());
    LOGGING_INFO(Gpu_voxels, "Calculating the sdf and gradient" << endl);
    pba_dist_map->parallelBanding3D();
    pba_inverse_dist_map->parallelBanding3D();
    HANDLE_CUDA_ERROR(cudaDeviceSynchronize());


    // Calculate the signed distance
    std::vector<float> sdf_map(pba_dist_map->getVoxelMapSize());
    pba_dist_map->getSignedDistancesToHost(pba_inverse_dist_map, sdf_map);
    
    // Calculate SDF and Gradients
    pba_dist_map->getSignedDistancesAndGradientsToHost(pba_inverse_dist_map, sdf_grad_map);
    std::cout << "...Signed distance field and gradients done" << std::endl;


    // Load known txt file
    std::vector<float> ReplayBuffer;
    std::ifstream in("/home/mark/code/sdf_package_testing/src/gpu_voxels_tester/src/gpu_voxels_test.txt");
    float MyArray[dimX][dimY][dimZ];
    std::vector<float> test_vec;
    for (size_t z = 0; z < dimZ; z++)
    {
        std::string line;
        getline(in, line);
        std::stringstream ss(line);


        for(int row = 0; row < dimX; ++row)
        {
            for(int col =0; col < dimY; ++col)
            {
                std::string el;
                getline(ss, el, ',');
                MyArray[col][row][z] = std::stof(el);
                // std::cout << el << std::endl;
                test_vec.push_back(std::stof(el));
            }
        }

    }
    in.close();
    std::cout << "Number of elements added: " << test_vec.size() << std::endl;


    gpu_voxels::Vector3ui map_dims(dimX, dimY, dimZ);
    //  Loop through elements and check
    for (size_t i = 0; i < sdf_grad_map.size(); i++)
    {
        gpu_voxels::Vector3i voxel_coords = voxelmap::mapToVoxelsSigned(i, map_dims); 

        if (voxel_coords.x > 0 && voxel_coords.x < dimX-1 && voxel_coords.y > 0 && voxel_coords.y < dimY-1 && voxel_coords.z > 0 && voxel_coords.z < dimZ-1){
            std::cout << "x: " << voxel_coords.x << "  y: " << voxel_coords.y << "  z: " << voxel_coords.z << "        "; // This is the sdf element
            // std::cout << "My sdf: " << sdf_grad_map[i].sdf << "      "<< std::endl; // This is the sdf element
            // std::cout << "My sdf: " << sdf_map[i]; // This is the sdf element
            std::cout << "My sdf: " << sdf_grad_map[i].sdf; // This is the sdf element
            std::cout << "   My Gradient: " << sdf_grad_map[i].x << "   " << sdf_grad_map[i].y << "   " << sdf_grad_map[i].z << std::endl;

            // std::cout << "   Matlab: " << MyArray[voxel_coords.x][voxel_coords.y][voxel_coords.z] << std::endl;
        }
    }
    

    
    LOGGING_INFO(Gpu_voxels, "File read successfully" << endl);
    LOGGING_INFO(Gpu_voxels, "shutting down" << endl);

    exit(EXIT_SUCCESS);
}