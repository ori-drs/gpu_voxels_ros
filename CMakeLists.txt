cmake_minimum_required(VERSION 3.0.2)
project(gpu_voxels_tester)

# Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} -O3 -Wall -pthread") # -Wextra -Werror
set(CMAKE_BUILD_TYPE "RELEASE")

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

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)


# Must be called before adding libraries
catkin_package(     INCLUDE_DIRS include
                    LIBRARIES gpu_voxels_tester
)

add_library(gpu_voxels_tester SHARED src/gpu_voxels_server.cpp)
target_link_libraries(gpu_voxels_tester 
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
)


add_definitions(
  ${icl_core_DEFINITIONS}
)


add_executable (mark_test src/mark_test.cpp)
target_link_libraries (mark_test
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

add_executable (mark_simple_test src/mark_simple_test.cpp)
target_link_libraries (mark_simple_test
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

#add_executable (gpu_voxels src/gpu_voxels.cpp)
## Link the executable to the library.
## We currently also have to link against Boost and icl_core...
#target_link_libraries (gpu_voxels
#    ${Boost_SYSTEM_LIBRARY}
#    ${icl_core_LIBRARIES}
#    ${gpu_voxels_LIBRARIES}
#    ${CUDA_LIBRARIES}
#    ${PCL_LIBRARIES}
#    ${CATKIN_LIBRARIES}
#    ${orocos_kdl_LIBRARIES}
#    ${roscpp_LIBRARIES}
#    ${rospy_LIBRARIES}
#
#)

# add_executable (distance_ros_demo src/DistanceROSDemo.cpp)
# # Link the executable to the library.
# # We currently also have to link against Boost and icl_core...
# target_link_libraries (distance_ros_demo
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


# add_executable(gpu_voxels_server_node
#   src/gpu_voxels_server_node.cpp
# )
# target_link_libraries(gpu_voxels_server_node 
#     ${roscpp_LIBRARIES}
# )




install(TARGETS gpu_voxels_tester
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})