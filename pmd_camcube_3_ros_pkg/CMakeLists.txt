cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

## check required prerequisites
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/external/cmake_modules")
FIND_PACKAGE(PMD REQUIRED)

## preprocessor definitions
ADD_DEFINITIONS(-DPMD_PLUGIN_DIR=\\"${PMD_PLUGIN_DIR}/\\")

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()



#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
include_directories( ${PMD_INCLUDE_DIR})


# add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

# PMD SDK Library
  rosbuild_add_library (pmd_sdk
                        src/sdk/pmdsdk2.cpp
                       )

rosbuild_add_executable(pmd_camcube_3_node src/pmd_camcube_3_node.cpp)
target_link_libraries(pmd_camcube_3_node pmd_sdk pmdaccess2)

