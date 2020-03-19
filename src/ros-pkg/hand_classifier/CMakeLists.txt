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

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
rosbuild_link_boost(${PROJECT_NAME} filesystem)

rosbuild_add_executable(extract_mbd src/extract_mbd.cpp)
target_link_libraries(extract_mbd ${PROJECT_NAME})

rosbuild_add_executable(visualize src/visualize.cpp)
target_link_libraries(visualize ${PROJECT_NAME})

rosbuild_add_executable(evaluate src/evaluate.cpp)
target_link_libraries(evaluate ${PROJECT_NAME})

rosbuild_add_executable(dump_graphviz src/dump_graphviz.cpp)
target_link_libraries(dump_graphviz ${PROJECT_NAME})
