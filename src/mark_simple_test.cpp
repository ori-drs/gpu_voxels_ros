#include <cstdlib>
#include <signal.h>

#include <gpu_voxels/GpuVoxels.h>
#include <gpu_voxels/helpers/GeometryGeneration.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <icl_core_config/Config.h>

#include <chrono>

using boost::dynamic_pointer_cast;
using boost::shared_ptr;
using gpu_voxels::voxelmap::ProbVoxelMap;
using gpu_voxels::voxelmap::DistanceVoxelMap;
using gpu_voxels::voxellist::CountingVoxelList;

shared_ptr<GpuVoxels> gvl;

Vector3ui map_dimensions(128, 128, 128);
float voxel_side_length = 0.01f; // 1 cm voxel size

void ctrlchandler(int)
{
  exit(EXIT_SUCCESS);
}

void killhandler(int)
{
  exit(EXIT_SUCCESS);
}


int main(int argc, char* argv[])
{
  signal(SIGINT, ctrlchandler);
  signal(SIGTERM, killhandler);

  icl_core::logging::initialize(argc, argv);
	  
  // Generate a GPU-Voxels instance:
  gvl = gpu_voxels::GpuVoxels::getInstance();
  gvl->initialize(map_dimensions.x, map_dimensions.y, map_dimensions.z, voxel_side_length);

  // std::vector<Vector3f> listPoints;
  // listPoints.push_back(Vector3f(30.0f, 9.0f, 12.0f));
  // listPoints.push_back(Vector3f(30.0f,15.0f, 12.0f));
  // listPoints.push_back(Vector3f(30.0f,21.0f, 12.0f));
  // listPoints.push_back(Vector3f(30.0f,27.0f, 12.0f));

  //PBA
  gvl->addMap(MT_DISTANCE_VOXELMAP, "pbaDistanceVoxmap");
  shared_ptr<DistanceVoxelMap> pbaDistanceVoxmap = dynamic_pointer_cast<DistanceVoxelMap>(gvl->getMap("pbaDistanceVoxmap"));

  gvl->addMap(MT_DISTANCE_VOXELMAP, "pbaInverseDistanceVoxmap");
  shared_ptr<DistanceVoxelMap> pbaInverseDistanceVoxmap = dynamic_pointer_cast<DistanceVoxelMap>(gvl->getMap("pbaInverseDistanceVoxmap"));


  gvl->addMap(MT_PROBAB_VOXELMAP, "erodeTempVoxmap1");
  shared_ptr<ProbVoxelMap> erodeTempVoxmap1 = dynamic_pointer_cast<ProbVoxelMap>(gvl->getMap("erodeTempVoxmap1"));

  gvl->addMap(MT_COUNTING_VOXELLIST, "countingVoxelList");
  shared_ptr<CountingVoxelList> countingVoxelList = dynamic_pointer_cast<CountingVoxelList>(gvl->getMap("countingVoxelList"));

  //PBA map clone for visualization without artifacts
  gvl->addMap(MT_DISTANCE_VOXELMAP, "pbaDistanceVoxmapVisual");
  shared_ptr<DistanceVoxelMap> pbaDistanceVoxmapVisual = dynamic_pointer_cast<DistanceVoxelMap>(gvl->getMap("pbaDistanceVoxmapVisual"));
  pbaDistanceVoxmapVisual->clearMap();


  LOGGING_INFO(Gpu_voxels, "start visualizing maps" << endl);
  while (true)
  {
    pbaDistanceVoxmap->clearMap();
    pbaInverseDistanceVoxmap->clearMap();
    countingVoxelList->clearMap();
    erodeTempVoxmap1->clearMap();

    // // Insert the CAMERA data (now in world coordinates) into the list
    Vector3f corner_min(0.10, 0.10, 0.10);
    Vector3f corner_max(1.10, 1.10, 1.10);

    Vector3f bottom_left_min(1.16, 1.16, 0.0);
    Vector3f bottom_left_max(1.26, 1.26, 0.1);    

    // Vector3f env_min(0,0,0);
    // Vector3f env_max(255, 255, 255);    
    // std::vector<Vector3f> envPoints = gpu_voxels::geometry_generation::createBoxOfPoints(env_min, env_max, voxel_side_length);
    // countingVoxelList->insertPointCloud(envPoints, eBVM_OCCUPIED);

    std::vector<Vector3f> listPoints = gpu_voxels::geometry_generation::createBoxOfPoints(corner_min, corner_max, voxel_side_length);
    countingVoxelList->insertPointCloud(listPoints, eBVM_OCCUPIED);

    // std::vector<Vector3f> listPoints2 = gpu_voxels::geometry_generation::createBoxOfPoints(bottom_left_min, bottom_left_max, voxel_side_length);
    // countingVoxelList->insertPointCloud(listPoints2, eBVM_OCCUPIED);

    // gvl->insertPointCloudIntoMap(listPoints, "pbaDistanceVoxmap", BitVoxelMeaning(1));

    // LOGGING_INFO(Gpu_voxels, "erode voxels into pbaDistanceVoxmap" << endl);
    erodeTempVoxmap1->merge(countingVoxelList);
    pbaDistanceVoxmap->mergeOccupied(erodeTempVoxmap1);
    pbaInverseDistanceVoxmap->mergeFree(erodeTempVoxmap1);

    // Calculate the distance map:
    LOGGING_INFO(Gpu_voxels, "calculate distance map for " << countingVoxelList->getDimensions().x << " occupied voxels" << endl);

    std::vector<gpu_voxels::VectorSdfGrad> sdf_grad_map(pbaDistanceVoxmap->getVoxelMapSize());
    LOGGING_INFO(Gpu_voxels, "Calculating the sdf and gradient" << endl);
    auto start = std::chrono::high_resolution_clock::now(); 
    pbaDistanceVoxmap->parallelBanding3D();
    pbaInverseDistanceVoxmap->parallelBanding3D();


    // // Calculate the signed distance
    // std::vector<float> sdf_map(pbaDistanceVoxmap->getVoxelMapSize());
    // pbaDistanceVoxmap->getSignedDistancesToHost(pbaInverseDistanceVoxmap, sdf_map);

    // float max = *std::max_element(sdf_map.begin(), sdf_map.end());
    // float min = *std::min_element(sdf_map.begin(), sdf_map.end());
    // LOGGING_INFO(Gpu_voxels, "Min SDF value: " << min << endl);
    // LOGGING_INFO(Gpu_voxels, "Max SDF value: " << max << endl);


    pbaDistanceVoxmap->getSignedDistancesAndGradientsToHost(pbaInverseDistanceVoxmap, sdf_grad_map);
    auto finish = std::chrono::high_resolution_clock::now(); 
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(finish - start); 
    double dur = duration.count();


    int id = voxelmap::getVoxelIndexSigned(map_dimensions, Vector3i(11, 61, 61));
    
    
    LOGGING_INFO(Gpu_voxels, "Gradients calculation time: " <<  dur << " microseconds." << endl);
    LOGGING_INFO(Gpu_voxels, "grad x: " <<  sdf_grad_map[id].x << endl);
    LOGGING_INFO(Gpu_voxels, "grad y: " <<  sdf_grad_map[id].y << endl);
    LOGGING_INFO(Gpu_voxels, "grad z: " <<  sdf_grad_map[id].z << endl);
    LOGGING_INFO(Gpu_voxels, "grad sdf: " <<  sdf_grad_map[id].sdf << endl);




    LOGGING_INFO(Gpu_voxels, "start cloning pbaDistanceVoxmap" << endl);
    // pbaDistanceVoxmapVisual->clone(*(pbaDistanceVoxmap.get()));
    pbaDistanceVoxmapVisual->clone(*(pbaInverseDistanceVoxmap.get()));
    LOGGING_INFO(Gpu_voxels, "done cloning pbaDistanceVoxmap" << endl);

    gvl->visualizeMap("pbaDistanceVoxmapVisual");
    // gvl->visualizeMap("pbaInverseDistanceVoxmap");
  }

  LOGGING_INFO(Gpu_voxels, "shutting down" << endl);

  exit(EXIT_SUCCESS);
}

