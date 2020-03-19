include ../../Makefile.conf

IFLAGS += -Wno-unused-parameter
LFLAGS += -ltrack_tools -lvelo_support $(INTERFACE_LIBS) $(GLOBAL_LIBS)

MODULE_NAME = EXTERNAL_CAMERA_PROJECTION

SOURCES = test_rectification.cpp calibrate_external_camera.cpp velodyne_log.cpp calibration_model.cpp calibration_view_controller.cpp epnp_solver.cpp \
		dump_images_from_video.cpp external_camera_model.cpp external_track_projector.cpp project_into_external_camera.cpp depth_data_viewer.cpp

PUBLIC_INCLUDES = velodyne_log.h
PUBLIC_LIBRARIES = libvelodyne_log.a
PUBLIC_BINARIES = 

TARGETS = test_rectification calibrate_external_camera dump_images_from_video external_track_projector project_into_external_camera depth_data_viewer

TEST_TARGETS = 
EVAL_TARGETS = 

ROS_DEPS = sensor_msgs opencv2 eigen_extensions camera_calibration_parsers image_geometry image_labeler epnp bag_of_tricks multibooster cluster_descriptors point_cloud_mapping sensor_msgs unconstrained_optimization serializable

# rules

depth_data_viewer: depth_data_viewer.o

libvelodyne_log.a: velodyne_log.o

project_into_external_camera: project_into_external_camera.o calibration_model.o epnp_solver.o libvelodyne_log.a

test_rectification: test_rectification.o

calibrate_external_camera: calibrate_external_camera.o calibration_view_controller.o calibration_model.o epnp_solver.o libvelodyne_log.a

dump_images_from_video: dump_images_from_video.o

external_track_projector: external_track_projector.o external_camera_model.o

include ../../Makefile.rules
