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

rosbuild_include(qt_build qt-ros)
rosbuild_prepare_qt4(QtNetwork QtCore)
rosbuild_add_boost_directories()
include_directories(${CMAKE_CURRENT_BINARY_DIR})

QT4_WRAP_CPP(RA_TCPSERVER_MOC include/remote_adventurer_server/RA_TcpServer.h)
QT4_WRAP_CPP(RA_TCPCLIENT_MOC src/test_client.h)

rosbuild_add_executable(remote_server src/remote_server.cpp src/RA_TcpServer.cpp ${RA_TCPSERVER_MOC})
target_link_libraries(remote_server ${QT_LIBRARIES})
rosbuild_link_boost(remote_server signals)

rosbuild_add_executable(test_client src/test_client.cpp ${RA_TCPCLIENT_MOC})
target_link_libraries(test_client ${QT_LIBRARIES})

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
