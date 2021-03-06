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
rosbuild_include(qt_build qt-ros)
rosbuild_prepare_qt4(QtNetwork QtCore QtGui)
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

QT4_WRAP_CPP(RA_REMOTECLIENTAPP_MOC include/remote_adventurer_client/RA_MainWindow.h
                                    include/remote_adventurer_client/RA_TcpClient.h
                                    include/remote_adventurer_client/RA_ConnectDialog.h
                                    include/remote_adventurer_client/RA_RemoteClientApp.h
                                    include/remote_adventurer_client/RA_WheelMeter.h
                                    include/remote_adventurer_client/RA_UltrasonicViewer.h
                                    include/remote_adventurer_client/RA_ControllerView.h)

rosbuild_add_executable(remote_client   src/main.cpp
                                        src/RA_MainWindow.cpp
                                        src/RA_TcpClient.cpp
                                        src/RA_ConnectDialog.cpp
                                        src/RA_RemoteClientApp.cpp
                                        src/RA_WheelMeter.cpp
                                        src/RA_UltrasonicViewer.cpp
                                        src/RA_ControllerView.cpp
                                        ${RA_REMOTECLIENTAPP_MOC})

target_link_libraries(remote_client ${QT_LIBRARIES})

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
