include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += -lipc_interface $(INTERFACE_LIBS) $(GLOBAL_LIBS)

LINK = g++

MODULE_NAME = 		RADAR_VIEW

SOURCES = 		radar_view.cpp radar_test.cc

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 	radar_view 

GRAPHICS_TARGETS = 	radar_view
TARGETS = 		radar_test 

# rules

radar_view:		radar_view.o

radar_test:		radar_test.o

include ../../Makefile.rules
