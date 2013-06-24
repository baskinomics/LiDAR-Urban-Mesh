cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(MY_GRAND_PROJECT)
find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization kdtree surface)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 
add_executable(read-las-ascii-write-pcd-test read-las-ascii-write-pcd.cpp)
target_link_libraries(read-las-ascii-write-pcd-test ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${PCL_KDTREE_LIBRARIES} ${PCL_SURFACE_LIBRARIES})