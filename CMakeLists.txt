cmake_minimum_required(VERSION 3.0.2)
project(auto_roomba)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(walker src/main.cpp src/roombaWalker.cpp)

target_link_libraries(walker
  ${catkin_LIBRARIES}
)