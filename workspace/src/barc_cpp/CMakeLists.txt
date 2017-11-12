cmake_minimum_required(VERSION 2.8.3)
project(barc_cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf
  cmake_modules
)

find_package(Eigen REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++11")

add_message_files(
  DIRECTORY msg
  FILES
  ECU.msg
  Encoder.msg
  Ultrasound.msg
  Z_KinBkMdl.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES barc_cpp
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(barc_cpp src/state_estimation_DynBkMdl.cpp src/ekf.hpp src/system_models.hpp)

target_link_libraries(barc_cpp
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)



