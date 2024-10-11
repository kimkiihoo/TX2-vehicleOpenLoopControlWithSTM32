cmake_minimum_required(VERSION 3.0.2)
project(kuruma_letgo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  Uwb_Location  # 添加对 Uwb_Location 包的依赖
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs Uwb_Location  # 添加 Uwb_Location 依赖
  # INCLUDE_DIRS include
  # LIBRARIES kuruma_letgo
  # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(kuruma_letgo_node src/kuruma_letgo.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(kuruma_letgo_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# (安装和测试部分如果不需要可以保持原样或根据需要修改)

#############
## Testing ##
#############
