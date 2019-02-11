cmake_minimum_required(VERSION 2.8.3)
project(lab1)
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)

catkin_package()

include_directories( include ${catkin_INCLUDE_DIRS} )

add_executable (moverobot src/moverobot.cpp)
target_link_libraries(moverobot ${catkin_LIBRARIES})

add_executable (drift src/drift.cpp)
target_link_libraries(drift ${catkin_LIBRARIES})

