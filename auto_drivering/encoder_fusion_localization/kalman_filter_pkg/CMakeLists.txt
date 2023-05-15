cmake_minimum_required(VERSION 3.0.2)
project(kalman_filter_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++14)
add_definitions(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  image_transport
  message_generation
  nav_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_msgs
  std_srvs
  tf
  eqyc_joy2can
)

add_message_files(
    FILES
    # IMU:
    IMUGNSSMeasurement.msg
    # lidar:
    LidarMeasurement.msg
    # ESKF covariance monitor:
    ESKFStd.msg
    # EKF covariance monitor:
    EKFStd.msg
    # synced GNSS-odo measurement:
    PosVel.msg
    # synced GNSS-odo-mag measurement:
    PosVelMag.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
   nav_msgs
)

find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package( Ceres REQUIRED)
find_package(GTSAM REQUIRED)
find_package(glog REQUIRED)

# set(CMAKE_MODULE_PATH "/home/luo/lib/pkg/glog-0.4.0")
# find_package(Gflags REQUIRED)
# find_package(Glog REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/GeographicLib)

# set(ALL_TARGET_LIBRARIES "")

# include(cmake/glog.cmake)
# include(cmake/YAML.cmake)
# include(cmake/PCL.cmake)
# include(cmake/eigen.cmake)
# # include(cmake/sophus.cmake)
# include(cmake/geographic.cmake)
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories("third_party/sophus")
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_DIRS}
  ${Eigen_DIRS}
  ${OpenCV_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${GFLAGS_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/third_party/GeographicLib/include/
  "third_party/sophus"
)

include_directories(BEFORE ${GLOG_INCLUDE_DIRS}) 

# include(cmake/global_defination.cmake)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fusion_localization
#  CATKIN_DEPENDS eigen_conversions geometry_msgs image_transport message_generation nav_msgs pcl_ros roscpp rospy sensor_msgs std_msgs std_msgs std_srvs tf
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/sensor_data/cloud_data.cpp
  src/sensor_data/gnss_data.cpp
  src/sensor_data/imu_data.cpp
  src/sensor_data/pose_data.cpp
  src/sensor_data/velocity_data.cpp
  src/sensor_data/key_frame.cpp

  src/subscriber/cloud_subscriber.cpp
  src/subscriber/odometry_subscriber.cpp
  src/subscriber/gnss_subscriber.cpp
  src/subscriber/imu_subscriber.cpp
  src/subscriber/pose_vel_subscriber.cpp
  src/subscriber/velocity_subscriber.cpp
  src/subscriber/pose_vel_subscriber.cpp

  src/tf_listener/tf_listener.cpp
  src/data_pretreat/data_pretreat_flow.cpp

  src/publisher/odometry_publisher.cpp
  src/publisher/cloud_publisher.cpp
  src/publisher/imu_publisher.cpp
  src/publisher/pos_vel_publisher.cpp
  src/publisher/key_frames_publisher.cpp

  src/models/cloud_filter/voxel_filter.cpp
  src/models/cloud_filter/box_filter.cpp
  
  src/models/registration/registration_icp.cpp
  src/models/registration/registration_ndt.cpp

  # /home/luo/work/filter_ws/src/kalman_filter_pkg/include/kalman_filter_pkg/sensor_data/state_data.hpp
  src/front_end/front_end_flow.cpp
  src/front_end/front_end.cpp

  src/matching/matching_flow.cpp
  src/matching/matching.cpp

  src/kalman_filter/kalman_filter_flow.cpp

  src/sensor_data/encoder_data.cpp
  src/subscriber/encoder_subscriber.cpp

  src/imu_encoder_eskf/imu_encoder_eskf.cpp

  src/imu_pre_integrator/imu_pre_integrator.cpp

  src/tools/file_manager.cpp
  src/tools/print_info.cpp

  src/graph_optimizer/ceres_graph_optimizer.cpp
  src/graph_optimizer/graph_optimizer_flow.cpp
  src/graph_optimizer/graph_optimizer.cpp

  src/gtsam_optimizer/gtsam_optimizer_flow.cpp
  src/gtsam_optimizer/gtsam_odom_optimizer_flow.cpp

)


add_executable(transfer_axis src/apps/transfer_axis.cpp )
target_link_libraries(transfer_axis
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${ALL_TARGET_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${CERES_LIBRARIES}
  libGeographiccc
  yaml-cpp
  -lglog
  gtsam
)

add_executable(gtsam_odom_optimizer_node src/apps/gtsam_odom_optimizer_node.cpp )

target_link_libraries(gtsam_odom_optimizer_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${ALL_TARGET_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${CERES_LIBRARIES}
  libGeographiccc
  yaml-cpp
  -lglog
  gtsam
)

add_executable(gtsam_optimizer_node src/apps/gtsam_optimizer_node.cpp )

target_link_libraries(gtsam_optimizer_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${ALL_TARGET_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${CERES_LIBRARIES}
  libGeographiccc
  yaml-cpp
  -lglog
  gtsam
)

add_executable(graph_optimizer_node src/apps/graph_optimizer_node.cpp )
target_link_libraries(graph_optimizer_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${ALL_TARGET_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${CERES_LIBRARIES}
  libGeographiccc
  yaml-cpp
  -lglog
  gtsam
)


# add_executable(front_end_node src/apps/front_end_node.cpp ${ALL_SRCS})
add_executable(kalman_filter_node src/apps/kalman_filter_node.cpp )
target_link_libraries(kalman_filter_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${ALL_TARGET_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${CERES_LIBRARIES}
  gtsam 
  glog::glog
  libGeographiccc
  yaml-cpp
  -lglog
  gtsam
)

add_executable(data_pretreat_node src/apps/data_pretreat_node.cpp )

# add_dependencies(front_end_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(data_pretreat_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${ALL_TARGET_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${CERES_LIBRARIES}
  gtsam 
  glog::glog
  libGeographiccc
  yaml-cpp
  -lglog
  gtsam
)

add_executable(matching_node src/apps/matching_node.cpp )
target_link_libraries(matching_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${ALL_TARGET_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${CERES_LIBRARIES}
  gtsam 
  glog::glog
  libGeographiccc
  yaml-cpp
  -lglog
  gtsam
)


add_executable(imu_encoder_eskf_node src/apps/imu_encoder_eskf_node.cpp )
target_link_libraries(imu_encoder_eskf_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${ALL_TARGET_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${CERES_LIBRARIES}
  gtsam 
  glog::glog
  libGeographiccc
  yaml-cpp
  -lglog
  gtsam
)

# add_executable(front_end_node src/apps/front_end_node.cpp )
# target_link_libraries(front_end_node
#   ${catkin_LIBRARIES}
#   ${PROJECT_NAME}
#   ${ALL_TARGET_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   ${Boost_LIBRARIES}
#   ${YAML_CPP_LIBRARIES}
#   ${GFLAGS_LIBRARIES}
#   ${GLOG_LIBRARIES}
#   ${CERES_LIBRARIES}
#   libGeographiccc
#   yaml-cpp
#   -lglog
# )