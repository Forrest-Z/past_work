cmake_minimum_required(VERSION 2.8.3)
project(gp_lio)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -O3")

find_package(catkin REQUIRED COMPONENTS
        nodelet
        geometry_msgs
        image_transport
        nav_msgs
        pcl_ros
        roscpp
        rospy
        sensor_msgs
        std_msgs
        tf
        message_generation
        )

find_package(GTSAM REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
find_package(Eigen3 REQUIRED QUIET)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
add_subdirectory(./ThirdParty/GeographicLib/)

message(STATUS "pcl version: ${PCL_VERSION}")
message(STATUS "GTSAM version: ${GTSAM_VERSION}")
message(STATUS "Eigen3 version: ${Eigen3_VERSION}")
message(STATUS "Ceres version: ${Ceres_VERSION}")
message(STATUS "opencv version: ${OpenCV_VERSION}")

message(STATUS "opencv libs: ${OpenCV_LIBS}")
message(STATUS "opencv include dir: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "opencv config path: ${OpenCV_CONFIG_PATH}")

add_message_files(FILES
        FeatureVector.msg
        )
generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES gp_lio
        CATKIN_DEPENDS nodelet geometry_msgs image_transport nav_msgs pcl_ros roscpp rospy sensor_msgs std_msgs tf message_runtime
        DEPENDS PCL
)

include_directories(
        include
        include/gp_lio
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIR}
        ${GPSLAM_INCLUDE_DIR}
        ${CERES_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ./ThirdParty/GeographicLib/include/
)

# build a shareable lib for the whole project
FILE(GLOB_RECURSE IncFiles "include/gp_lio/*")
set(SOURCE_FILES
        src/gp_lio/estimator/Estimator.cpp
        src/gp_lio/estimator/Parameters.cpp
        src/gp_lio/factor/GpPriorFactor.cpp
        src/gp_lio/factor/ImuFactor.cpp
        src/gp_lio/factor/LidarFactor.cpp
        src/gp_lio/factor/TwistFactor.cpp
        src/gp_lio/factor/PoseLocalParameterization.cpp
        src/gp_lio/factor/MarginalizationFactor.cpp
        src/gp_lio/feature/ImuPreintegration.cpp
        src/gp_lio/feature/LidarFeature.cpp
        src/gp_lio/feature/LidarHandcraftFeature.cpp
        src/gp_lio/mapper/Frame.cpp
        src/gp_lio/mapper/GlobalMapper.cpp
        src/gp_lio/mapper/KeyFrame.cpp
        src/gp_lio/mapper/LocalMapper.cpp
        src/gp_lio/mapper/LocalMapMatcher.cpp
        src/gp_lio/visualization/Visualization.cpp
        src/gp_lio/initial/StaticInitializer.cpp
        src/gp_lio/utility/SceneMainDirExtractor.cpp
        src/gp_lio/utility/file_manager.cpp
        src/gp_lio/mapper/globalOpt.cpp
        )
add_library(gp_lio SHARED ${SOURCE_FILES} ${IncFiles})
add_dependencies(gp_lio  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gp_lio ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} gtsam libGeographiccc)

# main nodelet
add_library(EstimatorNodelet src/gp_lio/nodelet/EstimatorNodelet.cpp)
add_dependencies(EstimatorNodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(EstimatorNodelet gp_lio)

# main node
add_executable(estimator_node src/gp_lio/nodelet/estimator_node.cpp)
target_link_libraries(estimator_node gp_lio EstimatorNodelet)

add_executable(localization_node
        src/gp_lio/localization/localization_node.cpp
        src/gp_lio/localization/lgi_based_localization.cpp
        src/gp_lio/localization/localization_flow.cpp
        src/gp_lio/localization/map_manager.cpp
        src/gp_lio/localization/tf_broadcaster.cpp
        src/gp_lio/localization/odom_publisher.cpp
        src/gp_lio/localization/parameters.cpp)
target_link_libraries(localization_node gp_lio ${catkin_LIBARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

# lpd_net
catkin_install_python(PROGRAMS scripts/lpd_net/lpdnet_node.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# build some examples for better coding
add_executable(plazaPose2 nodes/plazaPose2.cpp)
target_link_libraries(plazaPose2 gtsam ${catkin_LIBRARIES})

add_executable(sampleMultiThreads src/ros_test/SampleMultiThreads.cpp)
target_link_libraries(sampleMultiThreads ${catkin_LIBRARIES})

add_executable(sampleCeres src/ros_test/SampleCeres.cpp)
target_link_libraries(sampleCeres gp_lio ${catkin_LIBRARIES} ${CERES_LIBRARIES})

add_executable(exampleIMU src/ros_test/ExampleIMU.cpp)
target_link_libraries(exampleIMU gp_lio ${catkin_LIBRARIES} ${CERES_LIBRARIES})

add_executable(coordinate_align src/gp_lio/nodelet/coordinate_align.cpp)
target_link_libraries(coordinate_align gp_lio ${catkin_LIBRARIES} ${CERES_LIBRARIES})

add_executable(localMatching src/ros_test/LocalMapMatching.cpp)
target_link_libraries(localMatching gp_lio ${catkin_LIBRARIES} ${CERES_LIBRARIES})

add_executable(pose_graph src/ros_test/pose_graph.cpp)
target_link_libraries(pose_graph gp_lio ${catkin_LIBRARIES} ${CERES_LIBRARIES})

add_library(SampleNodelet src/ros_test/SampleNodelet.cpp)
add_dependencies(SampleNodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(SampleNodelet gtsam ${catkin_LIBRARIES})
