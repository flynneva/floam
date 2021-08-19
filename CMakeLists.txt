cmake_minimum_required(VERSION 3.2.1)
project(floam)

set(CMAKE_BUILD_TYPE "Release")

# PCL requires C++ 14 and above
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  tf2
  tf2_geometry_msgs
  eigen_conversions
  nodelet
  pcl_ros
)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
  include
  ${PCL_LIBRARY_DIRS}
  ${CERES_LIBRARY_DIRS}
)

catkin_package(
  DEPENDS EIGEN3 PCL Ceres 
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  nodelet
  tf2
  tf2_geometry_msgs
  eigen_conversions
  rosbag
  sensor_msgs
  pcl_ros
)

# reusable lidar library, provides the Lidar template class
add_library(${PROJECT_NAME}_lidar
  src/lidar.cpp
  src/lidar_utils.cpp
)

add_library(${PROJECT_NAME}_lidar_node
  src/lidar_imager_node.cpp
  src/lidar_scanner_node.cpp
)

target_link_libraries(${PROJECT_NAME}_lidar_node
  ${PROJECT_NAME}_lidar
  ${catkin_LIBRARIES}  
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
)

add_library(${PROJECT_NAME}_odom
  src/odom_estimation.cpp
  src/lidar_utils.cpp
  src/lidar_optimization.cpp
)

add_library(${PROJECT_NAME}_odom_node
  src/odom_estimation_node.cpp
)

target_link_libraries(${PROJECT_NAME}_odom_node
  ${PROJECT_NAME}_odom
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
)

add_library(${PROJECT_NAME}_mapping
  src/lidar_mapping.cpp
)

target_link_libraries(${PROJECT_NAME}_mapping
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES} 
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
)

add_library(${PROJECT_NAME}_mapping_node
  src/lidar_mapping_node.cpp
)

target_link_libraries(${PROJECT_NAME}_mapping_node
  ${PROJECT_NAME}_mapping
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES} 
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS
  ${PROJECT_NAME}_lidar
  ${PROJECT_NAME}_lidar_node
  ${PROJECT_NAME}_odom
  ${PROJECT_NAME}_odom_node
  ${PROJECT_NAME}_mapping
  ${PROJECT_NAME}_mapping_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
