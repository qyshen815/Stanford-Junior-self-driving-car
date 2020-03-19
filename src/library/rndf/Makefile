include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 

LINK = g++

MODULE_NAME = RNDF

SOURCES = mdf.cpp
SOURCES += rndfRoadNetwork.cpp rndfSegment.cpp rndfLane.cpp rndfWayPoint.cpp \
		   rndfZone.cpp rndfSpot.cpp rndfCrossWalk.cpp rndfTrafficLight.cpp rndfStringTools.cpp

PUBLIC_INCLUDES = mdf.h
PUBLIC_INCLUDES += rndf.h rndfTokens.h rndfRoadNetwork.h rndfSegment.h rndfLane.h rndfWayPoint.h \
		   		   rndfZone.h rndfSpot.h rndfCrossWalk.h rndfTrafficLight.h
		   		   
PUBLIC_LIBRARIES = librndf.a libmdf.a
PUBLIC_BINARIES = 

TARGETS = librndf.a libmdf.a 

# rules

libmdf.a:	mdf.o rndfStringTools.o

librndf.a:	rndfRoadNetwork.o rndfSegment.o rndfLane.o rndfWayPoint.o \
			rndfZone.o rndfSpot.o rndfCrossWalk.o rndfTrafficLight.o rndfStringTools.o

include ../../Makefile.rules
