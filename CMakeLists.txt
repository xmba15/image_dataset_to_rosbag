cmake_minimum_required(VERSION 3.10)
project(image_dataset_to_rosbag)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  camera_info_manager
  image_geometry
  roscpp
  sensor_msgs
  std_msgs
  rosbag
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    cv_bridge
    camera_info_manager
    image_geometry
    roscpp
    sensor_msgs
    std_msgs
    rosbag
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(mono_bag_converter_node
  ${PROJECT_SOURCE_DIR}/src/mono_bag_converter_node.cpp
  ${PROJECT_SOURCE_DIR}/src/mono_bag_converter.cpp
  ${PROJECT_SOURCE_DIR}/src/bag_converter_base.cpp
)
target_link_libraries(mono_bag_converter_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(stereo_bag_converter_node
  ${PROJECT_SOURCE_DIR}/src/stereo_bag_converter_node.cpp
  ${PROJECT_SOURCE_DIR}/src/stereo_bag_converter.cpp
  ${PROJECT_SOURCE_DIR}/src/bag_converter_base.cpp
)
target_link_libraries(stereo_bag_converter_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
