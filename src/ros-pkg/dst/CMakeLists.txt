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

rosbuild_add_library(${PROJECT_NAME}
  src/lib/potentials_cache.cpp
  src/lib/kinect_sequence.cpp
  src/lib/lockable.cpp
  src/lib/seed_npg.cpp
  src/lib/color_histogram_npg.cpp
  src/lib/seed_image_distance_npg.cpp
  src/lib/canny_epg.cpp
  src/lib/color_delta_epg.cpp
  src/lib/depth_epg.cpp
  src/lib/node_potential_aggregator.cpp
  src/lib/edge_potential_aggregator.cpp
  src/lib/node_potential_generator.cpp
  src/lib/edge_potential_generator.cpp
  src/lib/depth_projector.cpp
  src/lib/optical_flow_node.cpp
  src/lib/label_flow_npg.cpp
  src/lib/kdtree_node.cpp
  src/lib/scene_alignment_node.cpp
  src/lib/scene_alignment_npg.cpp
  src/lib/surface_normal_node.cpp
  src/lib/surface_normal_epg.cpp
  src/lib/bilateral_npg.cpp
  src/lib/helper_functions.cpp
  src/lib/image_region_iterator.cpp
  src/lib/organized_surface_normal_node.cpp
  src/lib/segmentation_visualizer.cpp
  src/lib/patch_classifier.cpp
  src/lib/depth_npg.cpp
  src/lib/image_processing_nodes.cpp
  src/lib/sequence_segmentation_view_controller.cpp
  src/lib/segmentation_pipeline.cpp
  )

rosbuild_add_executable(image_cut src/program/image_cut.cpp)

rosbuild_add_executable(kinect_cut src/program/kinect_cut.cpp)
target_link_libraries(kinect_cut ${PROJECT_NAME})

rosbuild_add_executable(segmentation_visualizer src/program/segmentation_visualizer.cpp)
target_link_libraries(segmentation_visualizer ${PROJECT_NAME})

rosbuild_add_executable(kinect_cut_interactive src/program/kinect_cut_interactive.cpp)
target_link_libraries(kinect_cut_interactive ${PROJECT_NAME})

rosbuild_add_executable(receive src/program/receive.cpp)
target_link_libraries(receive ${PROJECT_NAME})

rosbuild_add_gtest(test_maxflow src/test/test_maxflow.cpp)
target_link_libraries(test_maxflow ${PROJECT_NAME})
rosbuild_add_gtest_build_flags(test_maxflow)

