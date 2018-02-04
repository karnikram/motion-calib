cmake_minimum_required(VERSION 2.8.3)
project(motion_calib)

find_package(catkin REQUIRED roscpp message_filters aruco_mapping)

catkin_package()

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(calc_transform src/calc.cpp)

target_link_libraries(calc_transform
   ${catkin_LIBRARIES}
 )