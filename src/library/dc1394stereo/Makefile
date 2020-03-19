include ../../Makefile.conf

MODULE_NAME = DC1394 STEREO

CXXFLAGS +=
IFLAGS   += 
LFLAGS   +=

ifeq (1,$(HAVE_DC1394))
  PUBLIC_INCLUDES = dc1394stereo.h
  PUBLIC_LIBRARIES = libdc1394stereo.a
  PUBLIC_BINARIES = 
  SOURCES = dc1394stereo.cpp
  TARGETS = libdc1394stereo.a
endif

# rules

libdc1394stereo.a:		dc1394stereo.o

include ../../Makefile.rules


