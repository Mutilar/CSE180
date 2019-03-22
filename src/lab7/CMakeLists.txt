cmake_minimum_required(VERSION 2.8.3)
project(lab7)
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs tf tf2_msgs tf2_ros actionlib laser_geometry)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable (cleanarea src/cleanarea.cpp)
target_link_libraries(cleanarea ${catkin_LIBRARIES})
