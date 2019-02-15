cmake_minimum_required(VERSION 2.8.3)
project(lab2)
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)

catkin_package()

include_directories( include ${catkin_INCLUDE_DIRS} )

add_executable (gotoposition src/gotoposition.cpp)
target_link_libraries(gotoposition ${catkin_LIBRARIES})

