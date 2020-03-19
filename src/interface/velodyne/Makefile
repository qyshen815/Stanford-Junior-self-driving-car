include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 

MODULE_NAME = VELODYNE

SOURCES = 		velodyne_shm_interface.cpp

PUBLIC_INCLUDES = 	velodyne_interface.h velodyne_shm_interface.h
PUBLIC_LIBRARIES = 	libvelodyne_interface.a 
PUBLIC_BINARIES = 

TARGETS = 		libvelodyne_interface.a

# rules

libvelodyne_interface.a:	velodyne_shm_interface.o

include ../../Makefile.rules
