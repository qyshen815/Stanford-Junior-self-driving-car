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

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME}
  src/lib/classification.cpp
  src/lib/dataset.cpp
  src/lib/dataset_splitter.cpp
  src/lib/dataset_slicer.cpp
  src/lib/projection_slicer.cpp
  src/lib/trainers.cpp
  src/lib/schedulers.cpp
  src/lib/synthetic_data_generator.cpp
  src/lib/cross_validator.cpp
  src/lib/params.cpp
  src/lib/ssl.cpp
  src/lib/evaluator.cpp
  )

#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
rosbuild_link_boost(${PROJECT_NAME} program_options)

rosbuild_add_executable(orthogonality_experiment src/program/orthogonality_experiment.cpp)
rosbuild_add_executable(test_serialization_methods src/program/test_serialization_methods.cpp)

rosbuild_add_executable(evaluate_supervised src/program/evaluate_supervised.cpp)
target_link_libraries(evaluate_supervised ${PROJECT_NAME})

rosbuild_add_executable(cross_validate src/program/cross_validate.cpp)
target_link_libraries(cross_validate ${PROJECT_NAME})

rosbuild_add_executable(timing_experiment src/program/timing_experiment.cpp)
target_link_libraries(timing_experiment ${PROJECT_NAME})

rosbuild_add_gtest(test_odontomachus src/test/test_odontomachus.cpp)
target_link_libraries(test_odontomachus ${PROJECT_NAME})

rosbuild_add_gtest(test_ssl src/test/test_ssl.cpp)
target_link_libraries(test_ssl ${PROJECT_NAME})

