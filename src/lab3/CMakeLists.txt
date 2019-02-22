cmake_minimum_required(VERSION 2.8.3)
project(lab3)
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs tf tf2_msgs)

catkin_package()

include_directories( include ${catkin_INCLUDE_DIRS} )

add_executable (transformations src/transformations.cpp)
target_link_libraries(transformations ${catkin_LIBRARIES})

