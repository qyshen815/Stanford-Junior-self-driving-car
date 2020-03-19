include ../../Makefile.conf

CXXFLAGS += -DRNDF_GL
#IFLAGS += `gsl-config --cflags`
IFLAGS += `pkg-config freetype2 --cflags`
IFLAGS += $(STANFORD_DRIVING_IFLAGS)

ifneq ($(ARCH), Darwin)
LFLAGS += -Wl,--whole-archive
endif

ifneq ($(ARCH), Darwin)
LFLAGS += -Wl,--no-whole-archive
endif

LFLAGS += $(STANFORD_DRIVING_LFLAGS)

MODULE_NAME = "AW RNDF"

SOURCES = aw_checkPoint.cpp aw_exit.cpp \
          aw_lane.cpp aw_laneSegment.cpp aw_netElement.cpp aw_perimeter.cpp \
          aw_perimeterPoint.cpp aw_roadNetwork.cpp aw_segment.cpp \
          aw_spot.cpp aw_stop.cpp aw_SpeedLimit.cpp \
          aw_StringTools.cpp aw_wayPoint.cpp aw_zone.cpp \
          aw_laneQuadTree.cpp aw_roadNetworkSearch.cpp \
          aw_intersection.cpp aw_trafficLight.cpp aw_crosswalk.cpp \
          aw_rndfgl.cpp aw_trafficLightGL.cpp

PUBLIC_INCLUDES = aw_checkPoint.h aw_exit.h \
				          aw_lane.h aw_laneSegment.h aw_netElement.h aw_perimeter.h \
				          aw_perimeterPoint.h aw_roadNetwork.h aw_segment.h \
				          aw_spot.h aw_stop.h aw_SpeedLimit.h\
				          aw_StringTools.h aw_wayPoint.h aw_zone.h aw_RNDFTokens.h \
				          aw_rndfgl.h aw_laneQuadTree.h aw_roadNetworkSearch.h \
				          aw_intersection.h aw_RndfId.h aw_crosswalk.h aw_trafficLight.h
				         
PUBLIC_LIBRARIES = libaw_rndf.a libaw_rndfgl.a
PUBLIC_BINARIES = 

TARGETS = libaw_rndf.a libaw_rndfgl.a

ROS_DEPS = cgal

# rules
libaw_rndf.a: aw_checkPoint.o aw_exit.o \
          aw_lane.o aw_laneSegment.o aw_netElement.o aw_perimeter.o \
          aw_perimeterPoint.o aw_roadNetwork.o aw_segment.o \
          aw_spot.o aw_stop.o aw_SpeedLimit.o \
          aw_StringTools.o aw_wayPoint.o aw_zone.o \
          aw_laneQuadTree.o aw_roadNetworkSearch.o \
          aw_intersection.o \
          aw_trafficLight.o aw_crosswalk.o

libaw_rndfgl.a: aw_rndfgl.o aw_trafficLightGL.o


include ../../Makefile.rules
