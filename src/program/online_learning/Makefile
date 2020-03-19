include ../../Makefile.conf

IFLAGS += -Wno-unused-parameter
LFLAGS += -ltrack_descriptors -ltrack_tools  $(GLOBAL_LIBS) 

MODULE_NAME = ONLINE_LEARNING

SOURCES = simple_test.cpp memory_dealloc_test.cpp project_cached_descriptors.cpp generate_raw_od.cpp label_and_project.cpp projector.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES =
PUBLIC_BINARIES = 

GRAPHICS_TARGETS = 
TARGETS = 	memory_dealloc_test project_cached_descriptors simple_test generate_raw_od \
		label_and_project

TEST_TARGETS = 
EVAL_TARGETS = 

ROS_DEPS = performance_statistics online_learning multibooster sensor_msgs cluster_descriptors point_cloud_mapping matplotlib_interface

# rules
simple_test: simple_test.o

memory_dealloc_test: memory_dealloc_test.o

project_cached_descriptors: project_cached_descriptors.o projector.o

label_and_project: label_and_project.o projector.o

generate_raw_od: generate_raw_od.o

include ../../Makefile.rules
