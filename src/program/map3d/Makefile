include ../../Makefile.conf

LFLAGS += -lipc_interface -lvelo_support $(INTERFACE_LIBS) $(GLOBAL_LIBS) -llltransform -lIL
#LFLAGS += -ljpeg.ipp -lippj -lippcore -lIL

LINK = g++

MODULE_NAME = MAP3D

SOURCES = map3d.cpp vlf_view_dynamic.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = map3d

GRAPHICS_TARGETS = vlf_view_dynamic
TARGETS = map3d

ROS_DEPS = octomap

map3d: map3d.o

vlf_view_dynamic: vlf_view_dynamic.o

include ../../Makefile.rules
