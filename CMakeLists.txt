cmake_minimum_required(VERSION 2.8.3)
project(ros1_learning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

## refarence : http://wiki.ros.org/ja/catkin/CMakeLists.txt

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  message_generation
  sensor_msgs
  geometry_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  SensorAD.msg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros1_learning
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  DEPENDS system_lib
 )

## launch file
install(DIRECTORY 
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## build include diretory
include_directories(include)
#add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})

#add_executable(pub_line_parameter_csi_cuda src/pub_line_parameter_CSI_CUDA.cpp)
#target_compile_options(pub_line_parameter_csi_cuda PRIVATE ${OpenMP_FLAGS})
#target_link_libraries(pub_line_parameter_csi_cuda ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_LIBS})


## Generate node (python)
#catkin_install_python(PROGRAMS 
#  scripts/joy_con.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
