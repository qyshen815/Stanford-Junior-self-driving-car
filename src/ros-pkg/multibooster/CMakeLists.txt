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

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


rosbuild_add_boost_directories()


rosbuild_add_library(multibooster multibooster.cpp multibooster_dataset.cpp weak_classifier_tree.cpp)
target_link_libraries(multibooster)	

rosbuild_add_executable(multitool multitool.cpp)
target_link_libraries(multitool multibooster)

rosbuild_download_test_data(http://cs.stanford.edu/people/teichman/test_data/multibooster/example.mb test/example.mb 750abf4c3e859816cfc3bdc941a80db2)
rosbuild_download_test_data(http://cs.stanford.edu/people/teichman/test_data/multibooster/example.mbd test/example.mbd 8dfbf204a6d440d519328213788e2e0f)

rosbuild_add_gtest(test/test_multibooster_dataset test/test_multibooster_dataset.cpp)
rosbuild_add_gtest_build_flags(test/test_multibooster_dataset)
target_link_libraries(test/test_multibooster_dataset multibooster)

rosbuild_add_gtest(test/test_multibooster test/test_multibooster.cpp TIMEOUT 300)
rosbuild_add_gtest_build_flags(test/test_multibooster)
target_link_libraries(test/test_multibooster multibooster)

rosbuild_add_executable(test/speed_test test/speed_test.cpp)
target_link_libraries(test/speed_test multibooster)	

#rosbuild_add_link_flags(multibooster "-lprofiler")
