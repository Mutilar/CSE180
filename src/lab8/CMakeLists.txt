cmake_minimum_required(VERSION 2.8.3)
project(lab8)
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs tf tf2_msgs tf2_ros actionlib laser_geometry)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable (averagepose  src/averagepose.cpp)
target_link_libraries(averagepose  ${catkin_LIBRARIES}) 

add_executable (move  src/move.cpp)
target_link_libraries(move  ${catkin_LIBRARIES})
