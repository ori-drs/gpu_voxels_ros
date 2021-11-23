cmake_minimum_required(VERSION 3.0.2)
project(gpu_voxels_ros)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -D_BUILD_GVL_WITH_URDF_SUPPORT_ -D_IC_BUILDER_GPU_VOXELS_URDF_ROBOT_)

# First we have to find our dependencies:
FIND_PACKAGE(CUDA REQUIRED)
FIND_PACKAGE(icl_core PATHS /home/mark/installs/gpu-voxels/export REQUIRED )
FIND_PACKAGE(gpu_voxels PATHS /home/mark/installs/gpu-voxels/export REQUIRED)
FIND_PACKAGE(Boost COMPONENTS system REQUIRED)
FIND_PACKAGE(PCL REQUIRED)
FIND_PACKAGE(orocos_kdl REQUIRED)
FIND_PACKAGE(roscpp REQUIRED)
FIND_PACKAGE(rospy REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
find_package(ompl REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs)


# Must be called before adding libraries
catkin_package(     INCLUDE_DIRS include 
                    LIBRARIES gpu_voxels_ros
)

add_library(gpu_voxels_ros  
src/gpu_voxels_hsr_server.cpp
src/composite_sdf_test.cpp
src/live_composite_sdf.cpp
src/utils.cpp
# include/timing.h
src/timing.cc
src/RecoveryPlanner.cpp
)

target_link_libraries(gpu_voxels_ros 
    ${Boost_SYSTEM_LIBRARY}
    ${icl_core_LIBRARIES}
    ${gpu_voxels_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${PCL_LIBRARIES}
    ${CATKIN_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${rospy_LIBRARIES}
    ${OMPL_LIBRARIES}
    )

include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${gpu_voxels_INCLUDE_DIRS}
    ${icl_core_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
    ${rospy_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    )


add_definitions(
  ${icl_core_DEFINITIONS}
)


# add_executable (load_cow_and_lady_ptcloud src/load_cow_and_lady_ptcloud.cpp)
# target_link_libraries (load_cow_and_lady_ptcloud
#     ${Boost_SYSTEM_LIBRARY}
#     ${icl_core_LIBRARIES}
#     ${gpu_voxels_LIBRARIES}
#     ${CUDA_LIBRARIES}
#     ${PCL_LIBRARIES}
#     ${CATKIN_LIBRARIES}
#     ${orocos_kdl_LIBRARIES}
#     ${roscpp_LIBRARIES}
#     ${rospy_LIBRARIES}
# )

add_executable (sdf_test src/sdf_test.cpp)
target_link_libraries (sdf_test
    ${Boost_SYSTEM_LIBRARY}
    ${icl_core_LIBRARIES}
    ${gpu_voxels_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${PCL_LIBRARIES}
    ${CATKIN_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${rospy_LIBRARIES}
)

add_executable (composite_sdf_test_node src/composite_sdf_test_node.cpp)
target_link_libraries (composite_sdf_test_node
    ${Boost_SYSTEM_LIBRARY}
    ${icl_core_LIBRARIES}
    ${gpu_voxels_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${PCL_LIBRARIES}
    ${CATKIN_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${rospy_LIBRARIES}
    gpu_voxels_ros
)

add_executable (live_composite_sdf_test_node src/live_composite_sdf_test_node.cpp)
target_link_libraries (live_composite_sdf_test_node
    ${Boost_SYSTEM_LIBRARY}
    ${icl_core_LIBRARIES}
    ${gpu_voxels_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${PCL_LIBRARIES}
    ${CATKIN_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${rospy_LIBRARIES}
    gpu_voxels_ros
)

add_executable (gpu_voxels_server_node src/gpu_voxels_server_node.cpp)
target_link_libraries (gpu_voxels_server_node
    ${Boost_SYSTEM_LIBRARY}
    ${icl_core_LIBRARIES}
    ${gpu_voxels_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${PCL_LIBRARIES}
    ${CATKIN_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${rospy_LIBRARIES}
    ${OMPL_LIBRARIES}
    gpu_voxels_ros
)

install(TARGETS gpu_voxels_ros
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})