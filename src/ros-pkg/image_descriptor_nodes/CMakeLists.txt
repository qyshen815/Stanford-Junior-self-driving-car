cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)



rosbuild_download_data(http://cs.stanford.edu/people/teichman/test_data/image_descriptor_nodes/img.png img.png adceb86164a744c5d4991585906c862a)

rosbuild_add_gtest(test_${PROJECT_NAME} src/test_${PROJECT_NAME}.cpp)
target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})

rosbuild_add_executable(test_${PROJECT_NAME}_speed src/test_${PROJECT_NAME}_speed.cpp)
target_link_libraries(test_${PROJECT_NAME}_speed ${PROJECT_NAME})