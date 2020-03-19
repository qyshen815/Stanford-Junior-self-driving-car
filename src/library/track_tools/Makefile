include ../../Makefile.conf

CXXFLAGS += -g
IFLAGS += -Wno-unused-parameter

#Cruft flags from perception.
LFLAGS += -law_rndf -law_geometry -lipc_interface -lfilter  \
          $(INTERFACE_LIBS) $(GLOBAL_LIBS) 
LFLAGS += `gsl-config --libs` `pkg-config ImageMagick --libs`
LFLAGS += -lcar_list -lvelo_support -llogio -lsockutils
LFLAGS += -lgrid
LFLAGS += $(GRAPHICS_LIBS)
LFLAGS += -llltransform -lsockutils -lCGAL -law_rndf -lapplanix_interface
LFLAGS += -llocalize_interface -lIL -lftgl #-lippcore
IFLAGS += `pkg-config --cflags libavformat libavutil libswscale freetype2` #This doesn't work on 9.10.
LFLAGS += `pkg-config --libs libavcodec libavformat libavutil libswscale freetype2`
IFLAGS += -I/usr/include/libavformat -I/usr/include/libavcodec -I/usr/include/libswscale #For 9.10.
IFLAGS += -I/usr/include/freetype2/
LFLAGS += -lfastjpeg -lladybug_video -lladybug_model -lladybug_frameparser -limage -lblf -lasync_writer -lglobal -lfastjpeg -lladybug_interface $(IPPLIBS) -ldc1394


MODULE_NAME = TRACK_TOOLS_LIBRARY

SOURCES = track_manager_cached_descriptors.cpp multibooster_support.cpp track_manager.cpp

PUBLIC_INCLUDES = track_manager.h multibooster_support.h track_manager_cached_descriptors.h
PUBLIC_LIBRARIES = libtrack_tools.a 
PUBLIC_BINARIES = 

GRAPHICS_TARGETS =  
TARGETS = libtrack_tools.a


TEST_TARGETS = test_track_manager
EVAL_TARGETS = 

ROS_DEPS = multibooster cgal google-perftools cluster_descriptors sensor_msgs point_cloud_mapping google-test image_descriptor_nodes

# rules

libtrack_tools.a: track_manager.o multibooster_support.o track_manager_cached_descriptors.o

test_track_manager: test_track_manager.o libtrack_tools.a

include ../../Makefile.rules
