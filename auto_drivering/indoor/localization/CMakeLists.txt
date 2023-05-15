cmake_minimum_required(VERSION 2.8.3)
project(localization)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -O3")
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        sensor_msgs
        std_msgs
        tf
        geometry_msgs
        image_transport
        nav_msgs
        pcl_ros
        )
find_package(GTSAM REQUIRED)
find_package(PCL REQUIRED)
#find_package(Eigen3 REQUIRED QUIET)
find_package(OpenCV 3.2.0 REQUIRED)

find_package(glog 0.4.0 REQUIRED)

message(STATUS "opencv version: ${OpenCV_VERSION}")
message(STATUS "opencv libs: ${OpenCV_LIBS}")
message(STATUS "opencv include dir: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "opencv config path: ${OpenCV_CONFIG_PATH}")


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${GTSAM_INCLUDE_DIRS}
        ${GLOG_INCLUDE_DIRS}

)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS  geometry_msgs image_transport nav_msgs pcl_ros roscpp rospy sensor_msgs std_msgs tf message_runtime
        DEPENDS PCL
)

# build a shareable lib for the whole project
FILE(GLOB_RECURSE IncFiles "include/*")
set(SOURCE_FILES
        src/file_manager.cpp
        src/Frame.cpp
        src/KeyFrame.cpp
        src/lgi_based_localization.cpp
        src/localization_flow.cpp
        src/map_manager.cpp
        src/odom_publisher.cpp
        src/parameters.cpp
        src/tf_broadcaster.cpp
        src/IMUpreintegration_node.cpp
        )
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${IncFiles})
add_dependencies(${PROJECT_NAME}  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}  ${OpenCV_LIBS} ${PCL_LIBRARIES} gtsam glog::glog)

add_executable(localization_node  src/localization_node.cpp)
target_link_libraries(localization_node ${catkin_LIBARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${PROJECT_NAME})

add_executable(IMUpreintegration_node  src/IMUpreintegration_node.cpp)
target_link_libraries(IMUpreintegration_node ${catkin_LIBARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${PROJECT_NAME})
