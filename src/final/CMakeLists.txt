cmake_minimum_required(VERSION 2.8.3)
project(final)
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs tf tf2_msgs tf2)

catkin_package()

include_directories( include ${catkin_INCLUDE_DIRS} )

add_executable (preprocess_map src/preprocess_map.cpp)
target_link_libraries(preprocess_map ${catkin_LIBRARIES})

add_executable (scan_map src/scan_map.cpp)
target_link_libraries(scan_map ${catkin_LIBRARIES})


