# Created by the script cgal_create_cmake_script
# This is the CMake script for compiling a CGAL application.


project( lidar-processing-library_ )

cmake_minimum_required(VERSION 2.6.2)
if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" VERSION_GREATER 2.6)
  if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION}" VERSION_GREATER 2.8.3)
    cmake_policy(VERSION 2.8.4)
  else()
    cmake_policy(VERSION 2.6)
  endif()
endif()

find_package(CGAL QUIET COMPONENTS Core )
find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization kdtree surface)

if ( CGAL_FOUND )

  include( ${CGAL_USE_FILE} )

  include( CGAL_CreateSingleSourceCGALProgram )
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  create_single_source_cgal_program( "read-las-ascii-write-pcd.cpp" )
  # create_single_source_cgal_program( "resampling.cpp" )
  # create_single_source_cgal_program( "t.cpp" )

  target_link_libraries(read-las-ascii-write-pcd
   ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${PCL_KDTREE_LIBRARIES} ${PCL_SURFACE_LIBRARIES})

else()
  
    message(STATUS "This program requires the CGAL library, and will not be compiled.")
  
endif()

