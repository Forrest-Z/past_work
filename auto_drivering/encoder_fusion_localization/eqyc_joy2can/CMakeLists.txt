cmake_minimum_required(VERSION 2.8.3)
project(eqyc_joy2can)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
SET(CMAKE_CXX_FLAGS "-std=c++0x")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation kvaser_interface roscpp rospy std_msgs tf visualization_msgs joy sensor_msgs)
find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(FILES
  eqyc_IMCU_msg.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )


catkin_package(
  CATKIN_DEPENDS
  message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES eqyc_joy2can
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
# ${catkin_INCLUDE_DIRS}
)


add_executable(eqyc_joy2can
  src/EQYC_UCCAN.cpp
  src/eqyc_joy2can.cpp
)
add_dependencies(eqyc_joy2can ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(eqyc_joy2can
  ${catkin_LIBRARIES}
)

add_executable(track_control
        src/EQYC_UCCAN.cpp
        src/track_control.cpp
        )
add_dependencies(track_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(track_control
        ${catkin_LIBRARIES}
        )
add_executable(Odometry_Test
        src/eqyc_joy2can_2.cpp
        )
add_dependencies(Odometry_Test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Odometry_Test
        ${catkin_LIBRARIES}
        )

add_executable(path_tracking src/lateral_stanley.cpp)
target_link_libraries(path_tracking ${catkin_LIBRARIES})

add_executable(GlobalPathPublisher src/global_path_publisher.cpp)
target_link_libraries(GlobalPathPublisher ${catkin_LIBRARIES})
