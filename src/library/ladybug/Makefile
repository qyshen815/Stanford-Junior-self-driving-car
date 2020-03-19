include ../../Makefile.conf

MODULE_NAME = LADYBUG

ifeq (1,$(HAVE_IPP))
CXXFLAGS +=
IFLAGS   += -I/usr/include/freetype2
LFLAGS   += -lfastjpeg -lblf

PUBLIC_INCLUDES = ladybug_video.h ladybug_frameparser.h ladybug_playback.h ladybug_model.h
PUBLIC_LIBRARIES = libladybug_video.a libladybug_frameparser.a libladybug_model.a
PUBLIC_BINARIES = 

SOURCES = ladybug_video.cpp ladybug_config.cpp ladybug_cg.cpp ladybug_gl.cpp ladybug_frameparser.cpp ladybug_playback.cpp ladybug_model.cpp

TARGETS = libladybug_video.a libladybug_frameparser.a libladybug_model.a get_binaries

else

TARGETS = nofastjpeg

endif

ROS_DEPS = opencv2 eigen_extensions 

# rules

nofastjpeg:		;@echo " --> Skip libladybug, no ipp needed for fastjpeg"

libladybug_video.a:		ladybug_video.o ladybug_config.o ladybug_cg.o ladybug_gl.o ladybug_playback.o

libladybug_frameparser.a:	ladybug_frameparser.o

.PHONY: get_binaries

get_binaries: ../../../param/ladybug_extrinsics \
	../../../param/ladybug_intrinsics.dat

# We don't want binaries in TARGETS because make clean will delete it.
libladybug_model.a:		ladybug_model.o

../../../param/ladybug_extrinsics:
	mkdir -p $@
	@echo " -- Downloading velo-ladybug extrinsics."
	wget http://cs.stanford.edu/group/driving/data/ladybug/velo_to_ladybug_cam0.eig -O $@/velo_to_ladybug_cam0.eig
	wget http://cs.stanford.edu/group/driving/data/ladybug/velo_to_ladybug_cam1.eig -O $@/velo_to_ladybug_cam1.eig
	wget http://cs.stanford.edu/group/driving/data/ladybug/velo_to_ladybug_cam2.eig -O $@/velo_to_ladybug_cam2.eig
	wget http://cs.stanford.edu/group/driving/data/ladybug/velo_to_ladybug_cam3.eig -O $@/velo_to_ladybug_cam3.eig
	wget http://cs.stanford.edu/group/driving/data/ladybug/velo_to_ladybug_cam4.eig -O $@/velo_to_ladybug_cam4.eig


# This thing is too big to put in the repo.
../../../param/ladybug_intrinsics.dat:
	@echo "  -- Downloading ladybug remap tables.  This may take a few minutes."
	wget http://cs.stanford.edu/group/driving/data/ladybug/ladybug_intrinsics.dat -O $@

wipe: clean
	-rm ../../../param/ladybug_intrinsics.dat
	-rm -rf ../../../param/ladybug_extrinsics

include ../../Makefile.rules


