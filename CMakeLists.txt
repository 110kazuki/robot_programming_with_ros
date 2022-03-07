cmake_minimum_required(VERSION 2.8.3)
project(mecanum_control)

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
  #cv_bridge
  image_transport
  sensor_msgs
  geometry_msgs
  genmsg
)
#OpenMP
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBS gomp)
endif()

find_package(OpenCV REQUIRED
  NO_MODULE
  #PATHS /usr/local/include/opencv4/
  PATHS /usr/include/opencv4/   #Jetpack4.4
)

MESSAGE(WARNING "prefix ${CMAKE_PREFIX_PATH}")    
MESSAGE(WARNING "version ${OpenCV_VERSION}")
MESSAGE(WARNING "install path ${OpenCV_INSTALL_PATH}") 
MESSAGE(WARNING "config path ${OpenCV_CONFIG_PATH}") # look at the output of this message
MESSAGE(WARNING "include dirs ${OpenCV_INCLUDE_DIRS}")
MESSAGE(WARNING "libs ${OpenCV_LIBS}")

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  DriveStatus.msg
  FourWheelsF.msg
  FourWheelsI.msg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mecanum_control
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  DEPENDS system_lib
  DEPENDS OpenMP
 )

## launch file
install(DIRECTORY 
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## build include diretory
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
#add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})


#add_executable(joy_control src/joy_control.cpp)
#target_link_libraries(joy_control ${catkin_LIBRARIES}  ${OpenMP_LIBS})

#add_executable(pub_line_parameter_csi_cuda src/pub_line_parameter_CSI_CUDA.cpp)
#target_compile_options(pub_line_parameter_csi_cuda PRIVATE ${OpenMP_FLAGS})
#target_link_libraries(pub_line_parameter_csi_cuda ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_LIBS})

#add_executable(image_capture src/image_capture_csi.cpp)
#target_compile_options(image_capture PRIVATE ${OpenMP_FLAGS})
#target_link_libraries(image_capture ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_LIBS})

## Generate node (python)
catkin_install_python(PROGRAMS 
  scripts/joy_con.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
