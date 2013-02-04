cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

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

# Need to pick up autogen'd ui files in build dir, so add the following
include_directories(${CMAKE_CURRENT_BINARY_DIR})

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_boost_directories()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/RA_Dashboard.cpp src/RA_Commander.cpp src/RA_Listener.cpp src/RA_XMLHelper.cpp)
rosbuild_link_boost(${PROJECT_NAME} signals)

#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(lib_sample src/lib_sample.cpp)
target_link_libraries(lib_sample ${PROJECT_NAME})

rosbuild_add_executable(adventurer_node src/adventurer_node.cpp)
target_link_libraries(adventurer_node ${PROJECT_NAME})

rosbuild_add_executable(teleop_node src/teleop_node.cpp)
target_link_libraries(teleop_node ${PROJECT_NAME})

