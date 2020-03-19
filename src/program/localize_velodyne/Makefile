include ../../../src/Makefile.conf 

CFLAGS +=
IFLAGS += 
LFLAGS += -lgrid -limage -lvelo_support $(INTERFACE_LIBS)

LFLAGS += -lIL

LFLAGS += $(IPP_LIBS)

MODULE_NAME = LOCALIZE_VELODYNE

SOURCES = localize_velodyne.cpp map_velodyne.cpp terrainmap.cpp \
	  veloclient.cpp test_localize_accuracy.cpp velo_calibrate.cpp generate_rndf.cpp

ROS_DEPS += opencv2

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = map_velodyne localize_velodyne velo_calibrate generate_rndf

GRAPHICS_TARGETS = localize_velodyne map_velodyne test_localize_accuracy velo_calibrate generate_rndf

localize_velodyne:	localize_velodyne.o terrainmap.o veloclient.o

generate_rndf:		generate_rndf.o terrainmap.o

map_velodyne:		map_velodyne.o terrainmap.o

velo_calibrate:		velo_calibrate.o

test_localize_accuracy:	test_localize_accuracy.o

include ../../../src/Makefile.rules
