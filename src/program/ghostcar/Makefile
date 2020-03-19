include ../../Makefile.conf

CXXFLAGS += 
IFLAGS += 
LFLAGS += $(INTERFACE_LIBS) $(GLOBAL_LIBS) -lipc_interface

#LFLAGS += -lprofiler

LINK = g++

MODULE_NAME = GHOSTCAR

SOURCES = ghostcar_playback.cc applanix2ghostcar.cc conv_to_gcar.cc

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = ghostcar_playback applanix2ghostcar

TARGETS = ghostcar_playback applanix2ghostcar conv_to_gcar

# rules

ghostcar_playback:	ghostcar_playback.o

applanix2ghostcar:	applanix2ghostcar.o

conv_to_gcar:		conv_to_gcar.o

include ../../Makefile.rules
