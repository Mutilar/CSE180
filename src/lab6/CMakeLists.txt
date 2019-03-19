cmake_minimum_required(VERSION 2.8.3)
project(lab6)
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs tf tf2_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable (navigate src/navigate.cpp)
target_link_libraries(navigate ${catkin_LIBRARIES})
