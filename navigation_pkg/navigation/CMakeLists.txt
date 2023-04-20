cmake_minimum_required(VERSION 2.8.3)
project(navigation)

# Set to Release in order to speed up the program significantly
set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  # base_local_planner
  # costmap_2d
  costmap_converter
  cmake_modules
  dynamic_reconfigure
  geometry_msgs
  interactive_markers
  message_generation
  # nav_core
  nav_msgs
  # mbf_costmap_core
  mbf_msgs
  roscpp
  std_msgs
  # pluginlib
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)
message(STATUS "System: ${CMAKE_SYSTEM}")
## System dependencies are found with CMake's conventions
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
message(STATUS "${CMAKE_MODULE_PATH}")
find_package(Boost REQUIRED COMPONENTS system thread graph)
find_package(SUITESPARSE REQUIRED)
find_package(G2O REQUIRED)
find_package(OpenCV REQUIRED)
# Eigen3 FindScript Backward compatibility (ubuntu saucy)
# Since FindEigen.cmake is deprecated starting from jade.
if (EXISTS "FindEigen3.cmake")
  find_package(Eigen3 REQUIRED)
  set(Eigen_INCLUDE_DIRS ${Eigen3_INCLUDE_DIRS})
elseif (EXISTS "FindEigen.cmake")
  find_package(Eigen REQUIRED)
elseif (EXISTS "FindEigen.cmake")
  message(WARNING "No findEigen cmake script found. You must provde one of them,
  e.g. by adding it to ${PROJECT_SOURCE_DIR}/cmake_modules.")
endif (EXISTS "FindEigen3.cmake")

set(EXTERNAL_INCLUDE_DIRS ${Eigen_INCLUDE_DIRS} ${SUITESPARSE_INCLUDE_DIRS} ${G2O_INCLUDE_DIR})
set(EXTERNAL_LIBS ${SUITESPARSE_LIBRARIES} ${G2O_LIBRARIES})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


## C++11 support
## Unfortunately, the 3d-party dependency libg2o requires c++11 starting from ROS Jade.
## Even if the ROS Jade specifications do not want c++11-only packages,
## we cannot compile without c++11 enabled. Another option would be to downgrade  
## libg2o third-party package.
## By now, if you do not want c++11, please refer to the ros indigo version.
IF(NOT MSVC)
  include(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support which is required 
  by linked third party packages starting from ROS Jade. Ignore this message for ROS Indigo.")
endif()
endif()
include_directories(
        /opt/ros/melodic/include
        /usr/include/eigen3
        ${SUITESPARSE_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
)
include_directories(
 include
 include/global_planner
 include/navi_master
 include/costmap
 include/PID_control
  ${catkin_INCLUDE_DIRS}
)
################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
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
add_message_files(
  FILES
  TrajectoryPointMsg.msg
  TrajectoryMsg.msg
  FeedbackMsg.msg
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
generate_messages(
   DEPENDENCIES
   geometry_msgs std_msgs costmap_converter
)

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/TebLocalPlannerReconfigure.cfg
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include ${EXTERNAL_INCLUDE_DIRS}
  LIBRARIES teb_local_planner ${EXTERNAL_LIBS}
  CATKIN_DEPENDS
	# base_local_planner
	# costmap_2d
	costmap_converter
	dynamic_reconfigure
	geometry_msgs
	interactive_markers
	message_runtime
	# nav_core
	nav_msgs
	# pluginlib
	roscpp
  # mbf_costmap_core
	std_msgs
        tf2
        tf2_ros
	visualization_msgs
  DEPENDS SUITESPARSE G2O
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  SYSTEM
include/teb_local_planner
  ${EXTERNAL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Build the teb_local_planner library
add_library(global_planner_lib
src/global_planner/a_star.cpp
# src/global_planner/global_planner_ros.cpp
# src/global_planner/hybrid_a_star.cpp
 src/global_planner/dijkstra.cpp
# src/global_planner/hybrid_dstar_lite.cpp
)

add_library(costmap2d_lib
src/costmap/costmap_2d.cpp
src/costmap/costmap_ros.cpp
# src/costmap/costmap_convert.cpp
)

add_library(teb_local_planner
   src/teb_local_planner/timed_elastic_band.cpp
   src/teb_local_planner/optimal_planner.cpp
   src/teb_local_planner/obstacles.cpp
   src/teb_local_planner/visualization.cpp
   src/teb_local_planner/recovery_behaviors.cpp
   src/teb_local_planner/teb_config.cpp
   src/teb_local_planner/homotopy_class_planner.cpp
   # src/teb_local_planner/teb_local_planner_ros.cpp
   src/teb_local_planner/graph_search.cpp
)


add_library(navigation_ros_lib
src/PID_control/pid.cpp
src/PID_control/seed_control.cpp
src/navi_master/navigation_ros.cpp 
src/navi_master/navi_state.cpp
src/navi_master/seed_line_pose.cpp
)

# Dynamic reconfigure: make sure configure headers are built before any node using them
add_dependencies(teb_local_planner ${PROJECT_NAME}_gencfg)
# Generate messages before compiling the lib
add_dependencies(teb_local_planner ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(teb_local_planner
   ${EXTERNAL_LIBS}
   ${catkin_LIBRARIES}
)



## Build additional executables




add_executable(test_optim_node src/teb_local_planner/test_optim_node.cpp)

target_link_libraries(test_optim_node
   teb_local_planner
    ${EXTERNAL_LIBS}
   ${catkin_LIBRARIES}
 )

 add_executable(navigation_node src/navi_master/navigation_node.cpp )
 target_link_libraries(navigation_node
 navigation_ros_lib
 costmap2d_lib
 global_planner_lib
 teb_local_planner
 ${OpenCV_LIBS}
 ${catkin_LIBRARIES}
 ${EXTERNAL_LIBS}
 )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination

#############
## Testing ##
#############



## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_teb_local_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
