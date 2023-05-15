cmake_minimum_required(VERSION 2.8)
project(odom_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  angles
)


catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES odom_pkg
#  CATKIN_DEPENDS geometry_msgs nav_msgs pcl_ros roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  # ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${YAML_CPP_INCLUDE_DIR}
)

# ## Declare a C++ library
# add_library(odom_flow 
#   src/odom_flow.cpp
# )
# target_include_directories(odom_flow PRIVATE ${INCLUDE_DIRS})
# target_link_libraries(odom_flow ${LINK_LIBRARIES})
# /home/luo/carto_ws/catkin_works/src/odom_pkg/src/odometry_crawler_node.cpp
# /home/luo/carto_ws/catkin_works/src/odom_pkg/src/image_node.cpp
add_executable(image_node src/image_node.cpp) 
target_link_libraries(image_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)


add_executable(odometry_crawler_node src/odometry_crawler_node.cpp) 
target_link_libraries(odometry_crawler_node
  ${catkin_LIBRARIES}
  yaml-cpp
)


add_executable(odometry_node src/odometry_node.cpp) 
target_link_libraries(odometry_node
  ${catkin_LIBRARIES}
  yaml-cpp
)

add_executable(${PROJECT_NAME}_node src/odom_node.cpp src/odom_flow.cpp) 
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  yaml-cpp
)
# add_dependencies(${PROJECT_NAME}_node   ${catkin_EXPORTED_TARGETS})
