include ../../Makefile.conf

CFLAGS += 
IFLAGS += 
LFLAGS += 		-lusbfind $(INTERFACE_LIBS) $(GLOBAL_LIBS) \
			-lipc_interface -lcan -lcanlib
LINK = g++

MODULE_NAME = 		RADAR

SOURCES = 		radar_ipc.cpp radar.cpp radarcore.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 	radar 

TARGETS = 		radar  

# rules

radar:		radar_ipc.o radarcore.o radar.o

candump:	candump.o

include ../../Makefile.rules
