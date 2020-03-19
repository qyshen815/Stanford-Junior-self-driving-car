include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 

MODULE_NAME = LADYBUG

SOURCES = 		ladybug_shm_interface.cpp ladybug_interface.cpp

PUBLIC_INCLUDES = 	ladybug_interface.h ladybug_shm_interface.h
PUBLIC_LIBRARIES = 	libladybug_interface.a 
PUBLIC_BINARIES = 

TARGETS = 		libladybug_interface.a

# rules

libladybug_interface.a:	ladybug_shm_interface.o ladybug_interface.o

include ../../Makefile.rules
