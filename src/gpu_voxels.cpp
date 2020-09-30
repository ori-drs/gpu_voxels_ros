#include <cstdlib>
#include <signal.h>

#include <gpu_voxels/GpuVoxels.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <icl_core_config/Config.h>

using boost::dynamic_pointer_cast;
using boost::shared_ptr;
using gpu_voxels::voxelmap::ProbVoxelMap;
using gpu_voxels::voxelmap::DistanceVoxelMap;
using gpu_voxels::voxellist::CountingVoxelList;





namespace gpu_voxels {

class Master {
 public:

    shared_ptr<GpuVoxels> gvl;

    Vector3ui map_dimensions(256, 256, 256);
    float voxel_side_length = 0.01f; // 1 cm voxel size

    bool new_data_received;
    PointCloud my_point_cloud;
    Matrix4f tf;

    void ctrlchandler(int)
    {
      exit(EXIT_SUCCESS);
    }

    void killhandler(int)
    {
      exit(EXIT_SUCCESS);
    }

    void PointcloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
    {
      std::vector<Vector3f> point_data;
      point_data.resize(msg->points.size());

      for (uint32_t i = 0; i < msg->points.size(); i++)
      {
        point_data[i].x = msg->points[i].x;
        point_data[i].y = msg->points[i].y;
        point_data[i].z = msg->points[i].z;
      }

      //my_point_cloud.add(point_data);
      my_point_cloud.update(point_data);

      // transform new pointcloud to world coordinates
      my_point_cloud.transformSelf(&tf);
      
      new_data_received = true;

      LOGGING_INFO(Gpu_voxels, "DistanceROSDemo camera callback. PointCloud size: " << msg->points.size() << endl);
    }

    void Master::ParamSetup() {

      icl_core::config::GetoptParameter points_parameter("points-topic:", "t",
                                                        "Identifer of the pointcloud topic");
      icl_core::config::GetoptParameter roll_parameter  ("roll:", "r",
                                                        "Camera roll in degrees");
      icl_core::config::GetoptParameter pitch_parameter ("pitch:", "p",
                                                        "Camera pitch in degrees");
      icl_core::config::GetoptParameter yaw_parameter   ("yaw:", "y",
                                                        "Camera yaw in degrees");
      icl_core::config::GetoptParameter voxel_side_length_parameter("voxel_side_length:", "s",
                                                                    "Side length of a voxel, default 0.01");
      icl_core::config::GetoptParameter filter_threshold_parameter ("filter_threshold:", "f",
                                                                    "Density filter threshold per voxel, default 1");
      icl_core::config::GetoptParameter erode_threshold_parameter  ("erode_threshold:", "e",
                                                                    "Erode voxels with fewer occupied neighbors (percentage)");
      icl_core::config::addParameter(points_parameter);
      icl_core::config::addParameter(roll_parameter);
      icl_core::config::addParameter(pitch_parameter);
      icl_core::config::addParameter(yaw_parameter);
      icl_core::config::addParameter(voxel_side_length_parameter);
      icl_core::config::addParameter(filter_threshold_parameter);
      icl_core::config::addParameter(erode_threshold_parameter);
      icl_core::logging::initialize(argc, argv);
      
      voxel_side_length = icl_core::config::paramOptDefault<float>("voxel_side_length", 0.01f);

      // setup "tf" to transform from camera to world / gpu-voxels coordinates

      //const Vector3f camera_offsets(2, 0, 1); // camera located at y=0, x_max/2, z_max/2
      const Vector3f camera_offsets(map_dimensions.x * voxel_side_length * 0.5f, -0.2f, map_dimensions.z * voxel_side_length * 0.5f); // camera located at y=-0.2m, x_max/2, z_max/2

      float roll = icl_core::config::paramOptDefault<float>("roll", 0.0f) * 3.141592f / 180.0f;
      float pitch = icl_core::config::paramOptDefault<float>("pitch", 0.0f) * 3.141592f / 180.0f;
      float yaw = icl_core::config::paramOptDefault<float>("yaw", 0.0f) * 3.141592f / 180.0f;
      tf = Matrix4f::createFromRotationAndTranslation(Matrix3f::createFromRPY(-3.14/2.0 + roll, 0 + pitch, 0 + yaw), camera_offsets);

      std::string point_cloud_topic = icl_core::config::paramOptDefault<std::string>("points-topic", "/hsrb/head_rgbd_sensor/depth_registered/points");
      LOGGING_INFO(Gpu_voxels, "DistanceROSDemo start. Point-cloud topic: " << point_cloud_topic << endl);

      int filter_threshold = icl_core::config::paramOptDefault<int>("filter_threshold", 0);
      std::cout << "Remove voxels containing less points than: " << filter_threshold << std::endl;

      float erode_threshold = icl_core::config::paramOptDefault<float>("erode_threshold", 0.0f);
      std::cout << "Erode voxels with neighborhood occupancy ratio less or equal to: " << erode_threshold << std::endl;
    }

  void Master::GVL_Setup(){
    // Generate a GPU-Voxels instance:
    gvl = gpu_voxels::GpuVoxels::getInstance();
    gvl->initialize(map_dimensions.x, map_dimensions.y, map_dimensions.z, voxel_side_length);
    
    //Vis Helper
    gvl->addPrimitives(primitive_array::ePRIM_SPHERE, "measurementPoints");

    //PBA
    gvl->addMap(MT_DISTANCE_VOXELMAP, "pbaDistanceVoxmap");
    shared_ptr<DistanceVoxelMap> pbaDistanceVoxmap = dynamic_pointer_cast<DistanceVoxelMap>(gvl->getMap("pbaDistanceVoxmap"));

    gvl->addMap(MT_PROBAB_VOXELMAP, "erodeTempVoxmap1");
    shared_ptr<ProbVoxelMap> erodeTempVoxmap1 = dynamic_pointer_cast<ProbVoxelMap>(gvl->getMap("erodeTempVoxmap1"));
    gvl->addMap(MT_PROBAB_VOXELMAP, "erodeTempVoxmap2");
    shared_ptr<ProbVoxelMap> erodeTempVoxmap2 = dynamic_pointer_cast<ProbVoxelMap>(gvl->getMap("erodeTempVoxmap2"));

    gvl->addMap(MT_COUNTING_VOXELLIST, "countingVoxelList");
    shared_ptr<CountingVoxelList> countingVoxelList = dynamic_pointer_cast<CountingVoxelList>(gvl->getMap("countingVoxelList"));

    gvl->addMap(MT_COUNTING_VOXELLIST, "countingVoxelListFiltered");
    shared_ptr<CountingVoxelList> countingVoxelListFiltered = dynamic_pointer_cast<CountingVoxelList>(gvl->getMap("countingVoxelListFiltered"));

    //PBA map clone for visualization without artifacts
    gvl->addMap(MT_DISTANCE_VOXELMAP, "pbaDistanceVoxmapVisual");
    shared_ptr<DistanceVoxelMap> pbaDistanceVoxmapVisual = dynamic_pointer_cast<DistanceVoxelMap>(gvl->getMap("pbaDistanceVoxmapVisual"));
    pbaDistanceVoxmapVisual->clearMap();

    // Define two measurement points:
    std::vector<Vector3i> measurement_points;
    measurement_points.push_back(Vector3i(40, 100, 50));
    measurement_points.push_back(Vector3i(160, 100, 50));
    gvl->modifyPrimitives("measurementPoints", measurement_points, 5);    
    
  }
    int main(int argc, char* argv[])
    {
      Master::ParamSetup();
      Master::GVL_Setup();
      
      ros::init(argc, argv, "distance_ros_demo");
      ros::NodeHandle nh;
      ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> >(point_cloud_topic, 1, PointcloudCallback);

      ros::Rate r(30);
      size_t iteration = 0;
      new_data_received = true; // call visualize on the first iteration


      LOGGING_INFO(Gpu_voxels, "start visualizing maps" << endl);
      while (ros::ok())
      {
        ros::spinOnce();

        LOGGING_DEBUG(Gpu_voxels, "START iteration" << endl);

        // visualize new pointcloud if there is new data
        if (new_data_received) 
        {
          new_data_received = false;
          iteration++;

          pbaDistanceVoxmap->clearMap();
          countingVoxelList->clearMap();
          countingVoxelListFiltered->clearMap();
          erodeTempVoxmap1->clearMap();
          erodeTempVoxmap2->clearMap();

          // Insert the CAMERA data (now in world coordinates) into the list
          countingVoxelList->insertPointCloud(my_point_cloud, eBVM_OCCUPIED);
          gvl->visualizeMap("countingVoxelList");

          countingVoxelListFiltered->merge(countingVoxelList);
          countingVoxelListFiltered->remove_underpopulated(filter_threshold);
          gvl->visualizeMap("countingVoxelListFiltered");

          LOGGING_INFO(Gpu_voxels, "erode voxels into pbaDistanceVoxmap" << endl);
          erodeTempVoxmap1->merge(countingVoxelListFiltered);
          if (erode_threshold > 0)
          {
            erodeTempVoxmap1->erodeInto(*erodeTempVoxmap2, erode_threshold);
          } else
          {
            erodeTempVoxmap1->erodeLonelyInto(*erodeTempVoxmap2); //erode only "lonely voxels" without occupied neighbors
          }
          pbaDistanceVoxmap->mergeOccupied(erodeTempVoxmap2);

          // Calculate the distance map:
          LOGGING_INFO(Gpu_voxels, "calculate distance map for " << countingVoxelList->getDimensions().x << " occupied voxels" << endl);
          pbaDistanceVoxmap->parallelBanding3D();

          LOGGING_INFO(Gpu_voxels, "start cloning pbaDistanceVoxmap" << endl);
          pbaDistanceVoxmapVisual->clone(*(pbaDistanceVoxmap.get()));
          LOGGING_INFO(Gpu_voxels, "done cloning pbaDistanceVoxmap" << endl);

          gvl->visualizeMap("pbaDistanceVoxmapVisual");
          gvl->visualizePrimitivesArray("measurementPoints");

          // For the measurement points we query the clearance to the closest obstacle:
          thrust::device_ptr<DistanceVoxel> dvm_thrust_ptr(pbaDistanceVoxmap->getDeviceDataPtr());
          for(size_t i = 0; i < measurement_points.size(); i++)
          {
            int id = voxelmap::getVoxelIndexSigned(map_dimensions, measurement_points[i]);

            //get DistanceVoxel with closest obstacle information
            // DistanceVoxel dv = dvm_thrust_ptr[id]; // worked before Cuda9
            DistanceVoxel dv; //get DistanceVoxel with closest obstacle information
            cudaMemcpy(&dv, (dvm_thrust_ptr+id).get(), sizeof(DistanceVoxel), cudaMemcpyDeviceToHost);

            float metric_free_space = sqrtf(dv.squaredObstacleDistance(measurement_points[i])) * voxel_side_length;
            LOGGING_INFO(Gpu_voxels, "Obstacle @ " << dv.getObstacle() << " Voxel @ " << measurement_points[i] << " has a clearance of " << metric_free_space << "m." << endl);
          }
        }

        r.sleep();
      }

      LOGGING_INFO(Gpu_voxels, "shutting down" << endl);

      exit(EXIT_SUCCESS);
    }

  } // Class def
} // Namespace