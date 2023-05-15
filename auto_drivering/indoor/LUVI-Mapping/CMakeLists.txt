cmake_minimum_required(VERSION 2.8.3)
project(luvi-mapping)
set(CMAKE_BUILD_TYPE "Release")
#SET(CMAKE_BUILD_TYPE "Debug")
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
#SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
		geometry_msgs
		nav_msgs
		sensor_msgs
		roscpp
		rospy
		rosbag
		std_msgs
		image_transport
		cv_bridge
		tf
	    nlink_parser
)

find_package(GTSAM REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

message("-- Detected PCL version: " ${PCL_VERSION})
message("-- Detected OpenCV version: " ${OpenCV_VERSION})


include_directories(
		include
		${catkin_INCLUDE_DIRS}
		${PCL_INCLUDE_DIRS}
		${CERES_INCLUDE_DIRS}
		${OpenCV_INCLUDE_DIRS}
		${GTSAM_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs nlink_parser
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)


add_executable(ascanRegistration src/scanRegistration.cpp)
target_link_libraries(ascanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(alaserOdometry src/laserOdometry.cpp)
target_link_libraries(alaserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(alaserMapping src/laserMapping.cpp)
target_link_libraries(alaserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(kittiHelper src/kittiHelper.cpp)
target_link_libraries(kittiHelper ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(globalOpt src/globalOptimization.cpp src/keyFrame.cpp src/utility/file_manger.cpp)
target_link_libraries(globalOpt ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS}  gtsam)

add_executable(uwbCalib src/uwb_calibration.cpp)
target_link_libraries(uwbCalib ${catkin_LIBRARIES} ${pcl_LIBRARIES} ${CERES_LIBRARIES})




