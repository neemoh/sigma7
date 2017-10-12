cmake_minimum_required(VERSION 2.8.3)
project(sigma)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf_conversions
)

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

catkin_package()


add_executable(master
        src/main_master.cpp
        src/SigmaDevice.cpp
        src/SigmaDevice.hpp)

target_link_libraries(master
        ${catkin_LIBRARIES}
        dhd
        drd
        libusb-1.0.so.0 -lrt
        pthread
        )






