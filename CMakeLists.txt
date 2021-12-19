cmake_minimum_required(VERSION 3.0.2)
project(racecar_camera)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
	nodelet
	pluginlib
)
find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

## onnx_node
cuda_add_executable(onnx_node
  src/infer_process.cpp
  src/bbox_infer_node.cpp
)

add_dependencies(onnx_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(onnx_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CUDA_cublas_LIBRARY}
  nvinfer_plugin
  nvinfer
)

add_executable(waypoint_follow src/waypoint_follow.cpp)

target_link_libraries(waypoint_follow
  ${catkin_LIBRARIES}
)
