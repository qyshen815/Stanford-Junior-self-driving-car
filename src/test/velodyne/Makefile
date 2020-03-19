include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += -lipc_interface -lvelo_support $(INTERFACE_LIBS) -lapplanix_interface -lparam_interface -lvelocore $(GLOBAL_LIBS)

LINK = g++

MODULE_NAME = VELODYNE

SOURCES = vlf_projector_example.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 

TARGETS = vlf_projector_example

# rules

vlf_projector_example:	vlf_projector_example.o 

include ../../Makefile.rules
