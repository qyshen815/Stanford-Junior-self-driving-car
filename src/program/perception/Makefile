#DEBUG = 1

include ../../Makefile.conf

# CXXFLAGS += -fopenmp -D_MULTITHREAD -D_NDEBUG

IFLAGS += -Wno-unused-parameter

LFLAGS += -law_rndf -law_geometry -lipc_interface -lfilter  -ltrack_tools \
          $(INTERFACE_LIBS) $(GLOBAL_LIBS) 
LFLAGS += -law_rndfgl -lrndfgl -lgl_support -ltextures -limage
LFLAGS += `gsl-config --libs` 
LFLAGS += -lcar_list -lvelo_support -llogio -lsockutils
LFLAGS += -lgrid 

LFLAGS += $(GRAPHICS_LIBS)

LFLAGS += -llltransform -lsockutils -lCGAL -law_rndf -lapplanix_interface
LFLAGS += -llocalize_interface -lIL -lftgl -lglobal #-lippcore

IFLAGS += `pkg-config --cflags libavformat libavutil libswscale freetype2` #This doesn't work on 9.10.
LFLAGS += `pkg-config --libs libavcodec libavformat libavutil libswscale freetype2`
IFLAGS += -I/usr/include/libavformat -I/usr/include/libavcodec -I/usr/include/libswscale #For 9.10.
IFLAGS += -I/usr/include/freetype2/

MODULE_NAME = PERCEPTION

SOURCES = perception.cpp ipc.cc velodyne.cc ibeo.cc \
	  ldlrs.cc utils.cc init.cc appl_history.cc line.cc ray_tracing.cc \
	  integration.cc scan.cc tracker.cc kalman_multitracker.cpp \
	  box.cc segment.cpp obstacle.cpp tracked_obstacle.cpp radar.cc \
	  laser_segment.cpp velodyne_rings.cpp \

# SOURCES += sample1.cc sample1_unittest.cc
SOURCES += box_test.cpp tracker_test.cpp obstacle_test.cpp # perception_test.cpp 
SOURCES += perception_eval.cpp 

PUBLIC_INCLUDES = perception_scan.h perception_types.h applanix_history.h perception_defines.h
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = perception

GRAPHICS_TARGETS = 
TARGETS =  perception #track_classifier track_stats check_mbd_duplicates \
		       perception_viz background_extractor track_dataset_extractor track_extractor \
		       track_projector

TEST_TARGETS = perception_test
EVAL_TARGETS = perception_eval

ROS_DEPS = multibooster cgal google-perftools cluster_descriptors pipeline sensor_msgs opencv2 point_cloud_mapping google-test linear_kalman_filter matplotlib_interface bag_of_tricks
# rules

sample1_unittest:	sample1.o sample1_unittest.o

perception: perception.o ipc.o velodyne.o \
            ldlrs.o utils.o init.o appl_history.o line.o ray_tracing.o \
            integration.o scan.o tracker.o kalman_multitracker.o \
            box.o segment.o obstacle.o tracked_obstacle.o radar.o \
            laser_segment.o velodyne_rings.o             
            
perception_eval: perception_eval.o 
   
#perception_test: perception_test.o box.o obstacle_test.o obstacle.o tracked_obstacle.o utils.o \
#                 tracker_test.o kalman_tracker.o appl_history.o radar.o velodyne.o 
            
perception_test: box_test.o box.o \
                 obstacle.o tracked_obstacle.o utils.o velodyne.o utils.o \
                 kalman_tracker.o radar.o appl_history.o velodyne_rings.o \


perception_viz:  perception_viz.o ipc.o velodyne.o \
                 ldlrs.o utils.o init.o appl_history.o line.o ray_tracing.o \
                 integration.o scan.o tracker.o kalman_tracker.o \
                 box.o segment.o obstacle.o tracked_obstacle.o radar.o \
                 laser_segment.o velodyne_rings.o

background_extractor:  background_extractor.o ipc.o velodyne.o \
                 ldlrs.o utils.o init.o appl_history.o line.o ray_tracing.o \
                 integration.o scan.o tracker.o kalman_tracker.o \
                 box.o segment.o obstacle.o tracked_obstacle.o radar.o \
                 laser_segment.o velodyne_rings.o

track_extractor:  track_extractor.o ipc.o velodyne.o \
                 ldlrs.o utils.o init.o appl_history.o line.o ray_tracing.o \
                 integration.o scan.o tracker.o kalman_tracker.o \
                 box.o segment.o obstacle.o tracked_obstacle.o radar.o \
                 laser_segment.o velodyne_rings.o track_manager.o

../../../param/classifiers/2010-05-03_hogspin_12cand_renamed.mb:
	wget http://cs.stanford.edu/people/teichman/classifiers/2010-05-03_hogspin_12cand_renamed.mb -O $@

wipe: clean
	-rm ../../../param/classifiers/rss2010_explicit_prior.mb
	-rm ../../../param/classifiers/2010-05-03_hogspin_12cand.mb

include ../../Makefile.rules
