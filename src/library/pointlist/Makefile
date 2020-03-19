include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 

MODULE_NAME = POINTLIST

SOURCES = pointlist.cpp facelist.cpp

PUBLIC_INCLUDES = pointlist.h facelist.h
PUBLIC_LIBRARIES = libpointlist.a libfacelist.a
PUBLIC_BINARIES = 

TARGETS = libpointlist.a libfacelist.a

# rules

libpointlist.a:		pointlist.o

libfacelist.a:		facelist.o

include ../../Makefile.rules
