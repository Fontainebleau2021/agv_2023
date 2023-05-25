cmake_minimum_required(VERSION 3.0.2)
set(PROJECT_NAME zed_open_capture)

project(zed-open-capture)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

# Required external libraries

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake")

# Generate libraries
find_package(LIBUSB 1 REQUIRED)
find_package(HIDAPI REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
)

# Sources
set(SRC_VIDEO
    src/videocapture.cpp
)

set(SRC_SENSORS
    src/sensorcapture.cpp
)

set(HEADERS_VIDEO
    include/videocapture.hpp
    include/defines.hpp
    include/videocapture_def.hpp
)

set(HEADERS_SENSORS
    include/sensorcapture.hpp
    include/defines.hpp
    include/sensorcapture_def.hpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${HIDAPI_INCLUDE_DIRS}
  ${LibUSB_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

add_definitions(-DSENSORS_MOD_AVAILABLE)


set(SRC_FULL ${SRC_FULL} ${SRC_SENSORS} ${SRC_VIDEO})
set(HDR_FULL ${HDR_FULL} ${HEADERS_SENSORS} ${HEADERS_VIDEO})
set(DEP_LIBS ${DEP_LIBS} ${LibUSB_LIBRARIES} ${HIDAPI_LIBRARIES} pthread)

add_definitions(-DVIDEO_MOD_AVAILABLE)

add_library(${PROJECT_NAME} SHARED ${SRC_FULL} )
target_link_libraries( ${PROJECT_NAME}  ${DEP_LIBS})

# Generate Video Example
add_executable(zed2_ros examples/zed_oc_video_example.cpp)
target_link_libraries(zed2_ros
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)

