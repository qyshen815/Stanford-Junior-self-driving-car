include ../../Makefile.conf

ROS_DEPS += opencv2

IFLAGS += `pkg-config --cflags freetype2`
LFLAGS += -lgrid -lvelo_support $(INTERFACE_LIBS) -lANN-old \
	  $(GLOBAL_LIBS) -lipc_interface 
LFLAGS += -lftgl -lIL

LFLAGS += $(IPP_LIBS)

LINK = g++

MODULE_NAME = SLAM

SOURCES = seek_scan.cpp coverage_view.cpp slam.cpp slam_gui.cc spins.cc \
	  intensitycal.cc slam_inputs.cc new_scanmatch.cc match_thread.cc \
	  overlap.cc correctindex.cc rewrite_logs.cc 

ROS_DEPS += opencv2

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = slam_gui slam_batch coverage_view

GRAPHICS_TARGETS = slam_gui slam_batch 
TARGETS = 	seek_scan

seek_scan:		seek_scan.o 

coverage_view:		coverage_view.o

generate_coverage_overlay:	generate_coverage_overlay.o slam_config.o 

slam_gui:		slam_gui.o spins.o intensitycal.o slam_inputs.o \
			new_scanmatch.o match_thread.o overlap.o  \
			slam.o correctindex.o rewrite_logs.o

slam_batch:		slam_batch.o spins.o intensitycal.o slam_inputs.o \
			new_scanmatch.o match_thread.o overlap.o  \
			slam.o correctindex.o rewrite_logs.o

include ../../Makefile.rules

