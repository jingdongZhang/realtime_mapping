cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(realtime_mapping)

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
endif()

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})
include (FindPkgConfig)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(G2O)

include_directories(include/ ${G2O_INCLUDE_DIRS})
link_directories(${G2O_LIBRARY_DIRS})
add_definitions(${G2O_DEFINITIONS})

add_executable (realtime_mapping src/realtime_mapping_main.cpp)
target_link_libraries (realtime_mapping cxsparse libg2o_core.so libg2o_stuff.so libg2o_types_slam3d.so libg2o_csparse_extension.so libg2o_solver_csparse.so  ${G2O_LIBRARIES} ${PCL_LIBRARIES})
