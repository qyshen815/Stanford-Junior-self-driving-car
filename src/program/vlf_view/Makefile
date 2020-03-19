include ../../Makefile.conf

ROS_DEPS += opencv2

CFLAGS +=
IFLAGS += `pkg-config --cflags freetype2`
LFLAGS += -lipc_interface -lvelo_support -lcar_list $(INTERFACE_LIBS) $(GLOBAL_LIBS)
LFLAGS += -lftgl -lIL

LFLAGS += $(IPP_LIBS)

LFLAGS += -lgraph_cut -lgrid

LINK = g++

MODULE_NAME = VLF_VIEW

SOURCES = vlf_view.cpp vlf_label.cpp point_label.cpp

ROS_DEPS += opencv2

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = vlf_view vlf_label point_label

GRAPHICS_TARGETS = vlf_view vlf_label point_label
TARGETS = 

# rules

vlf_view:	vlf_view.o

vlf_label:	vlf_label.o

point_label: point_label.o

include ../../Makefile.rules
