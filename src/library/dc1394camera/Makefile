include ../../Makefile.conf

MODULE_NAME = DC1394CAMERA

CXXFLAGS +=
IFLAGS   += 
LFLAGS   +=

ifeq (1,$(HAVE_DC1394))
  PUBLIC_INCLUDES = dc1394camera.h
  PUBLIC_LIBRARIES = libdc1394camera.a
  PUBLIC_BINARIES = 
  SOURCES = dc1394camera.cc
  TARGETS = libdc1394camera.a
endif

# rules

libdc1394camera.a:		dc1394camera.o

include ../../Makefile.rules


