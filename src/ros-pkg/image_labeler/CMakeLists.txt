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


rosbuild_add_library(image_label_manager src/image_label_manager.cpp)
rosbuild_add_boost_directories()
rosbuild_link_boost(image_label_manager filesystem)

rosbuild_add_library(image_labeler_components src/opencv_view.cpp src/image_labeler_controller.cpp src/oneoff_image_labeler_controller.cpp)
target_link_libraries(image_labeler_components image_label_manager)

rosbuild_add_executable(dataset_viewer src/dataset_viewer.cpp)
target_link_libraries(dataset_viewer image_label_manager)

rosbuild_add_executable(label_sampler src/label_sampler.cpp)
target_link_libraries(label_sampler image_label_manager)

rosbuild_add_executable(image_labeler src/image_labeler.cpp)
target_link_libraries(image_labeler image_labeler_components)

rosbuild_add_executable(dump_objects src/dump_objects.cpp)
target_link_libraries(dump_objects image_label_manager)

rosbuild_add_executable(cropped_image_viewer src/cropped_image_viewer.cpp)
target_link_libraries(cropped_image_viewer image_label_manager)

# This is the right way to do it, but we're not using a version of ROS that supports this yet.
#rosbuild_download_test_data(http://cs.stanford.edu/people/teichman/test_data/image_labeler/example_run.tar.gz example_run.tar.gz)
#rosbuild_untar_file(example_run.tar.gz example_run tests)http://cs.stanford.edu/people/teichman/test_data/image_labeler/example_run.tar.gz example_run.tar.gz)

rosbuild_download_test_data(http://cs.stanford.edu/people/teichman/test_data/image_labeler/example_run/images/cam1_1284747777.426767.png example_run/images/cam1_1284747777.426767.png 8b7f70586ff357a8f427a5dee1f495f2)
rosbuild_download_test_data(http://cs.stanford.edu/people/teichman/test_data/image_labeler/example_run/images/cam1_1284747777.533003.png example_run/images/cam1_1284747777.533003.png 346ef026c2b22ebcb11ea9584c661d90)
rosbuild_download_test_data(http://cs.stanford.edu/people/teichman/test_data/image_labeler/example_run/images/cam1_1284747777.639343.png example_run/images/cam1_1284747777.639343.png 213329cefb13694ac24bc72ed00b20fb)

rosbuild_add_gtest(test_image_label_manager src/test_image_label_manager.cpp)
target_link_libraries(test_image_label_manager image_label_manager)
