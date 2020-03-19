include ../../Makefile.conf

IFLAGS += -Wno-unused-parameter
LFLAGS += -ltrack_tools $(GLOBAL_LIBS) 

MODULE_NAME = TRACK_DESCRIPTORS_LIBRARY

SOURCES = track_descriptors.cpp test_track_descriptors.cpp

PUBLIC_INCLUDES = track_descriptors.h
PUBLIC_LIBRARIES = libtrack_descriptors.a 
PUBLIC_BINARIES = 

GRAPHICS_TARGETS =  
TARGETS = libtrack_descriptors.a


TEST_TARGETS = test_track_descriptors
EVAL_TARGETS = 

ROS_DEPS = pipeline sensor_msgs google-test cluster_descriptors point_cloud_mapping

# rules

libtrack_descriptors.a: track_descriptors.o
test_track_descriptors: test_track_descriptors.o libtrack_descriptors.a

include ../../Makefile.rules
