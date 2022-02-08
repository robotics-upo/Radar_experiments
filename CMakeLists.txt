cmake_minimum_required(VERSION 2.8.3)
project(radar_experiments)

## Compile as C++11, supported in ROS Kinetic and newer
set (CMAKE_CXX_STANDARD 14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  # aruco_detect
  # fiducial_slam
  # flip_image
  # gather_data
  # image_transport
  # mcl3d
  dynamic_reconfigure
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  pcl_ros
)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(PCL REQUIRED)
find_package(G2O REQUIRED)
find_package(Armadillo REQUIRED)
# find_package(Eigen REQUIRED)
set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/LidarFusion.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES radar_experiments
#  CATKIN_DEPENDS aruco_detect fiducial_slam flip_Image gather_data image_transport mcl3d roscpp rospy sensor_msgs
  DEPENDS G2O
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
  src
  ${G2O_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR_EXTERNAL}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(lidar_std_dev_node src/sensor_fusion/lidar_std_dev_node.cpp)
target_link_libraries(lidar_std_dev_node ${catkin_LIBRARIES})
add_dependencies(lidar_std_dev_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(lidar_radar_fusion_node src/sensor_fusion/lidar_radar_fusion.cpp)
target_link_libraries(lidar_radar_fusion_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(lidar_radar_fusion_node ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(lidar_radar_dbscan src/sensor_fusion/lidar_radar_fusion_dbscan.cpp src/dbscan/dbscan.cpp src/dbscan/dbscan_lines.cpp
          src/dbscan/line.cpp src/dbscan/detected_line.cpp
)
target_link_libraries(lidar_radar_dbscan ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(lidar_radar_basic_fusion src/sensor_fusion/lidar_radar_fusion_basic_fusion.cpp)
target_link_libraries(lidar_radar_basic_fusion ${catkin_LIBRARIES} ${PCL_LIBRARIES})
# add_dependencies(lidar_radar_fusion_gm ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(save_lidar_radar src/generate_dataset/save_radar_lidar.cpp)
target_link_libraries(save_lidar_radar ${catkin_LIBRARIES} )

add_executable(refine_gt src/refine_gt.cpp)
target_link_libraries(refine_gt ${catkin_LIBRARIES} ${G2O_LIBRARIES})
# add_dependencies(lidar_radar_fusion_node ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

