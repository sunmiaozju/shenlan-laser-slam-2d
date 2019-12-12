cmake_minimum_required( VERSION 2.8 )
project( odom_calib )

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR} )

include_directories( "/usr/include/eigen3" )

add_executable( odom_calib odom_calib.cpp )
