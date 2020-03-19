include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 
LINK = g++

MODULE_NAME = 		CAMERA

SOURCES = 		camera_shm_interface.cpp camera_interface.cpp

PUBLIC_INCLUDES = 	camera_interface.h camera_shm_interface.h
PUBLIC_LIBRARIES = 	libcamera_interface.a
PUBLIC_BINARIES = 

TARGETS = 		libcamera_interface.a 

# rules

libcamera_interface.a:	camera_shm_interface.o camera_interface.o

include ../../Makefile.rules
