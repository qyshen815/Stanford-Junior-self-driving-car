include ../../Makefile.conf

DEBUG=0

ifndef DEBUG
CXXFLAGS += -O3 -w
else
CXXFLAGS += -O0 -g -w
endif

CFLAGS += 
IFLAGS += 
LFLAGS += -lrndf -lglobal

LINK = g++

MODULE_NAME = RNDF SEARCH

SOURCES = rndf_search.cpp rndf_lane_quadtree.cpp rndf_waypoint_quadtree.cpp geometry_2d.cpp

PUBLIC_INCLUDES = rndf_types.h rndf_lane_quadtree.h rndf_search.h rndf_waypoint_quadtree.h
PUBLIC_LIBRARIES = librndf_search.a
PUBLIC_BINARIES =

TARGETS = librndf_search.a

# rules

librndf_search.a: rndf_search.o rndf_lane_quadtree.o rndf_waypoint_quadtree.o geometry_2d.o

include ../../Makefile.rules
