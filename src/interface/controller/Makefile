include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 

MODULE_NAME = 		CONTROLLER

SOURCES = 		controller_interface.cc

PUBLIC_INCLUDES = 	controller_messages.h controller_interface.h
PUBLIC_LIBRARIES = 	libcontroller_interface.a
PUBLIC_BINARIES = 

TARGETS = 		libcontroller_interface.a

# rules

libcontroller_interface.a:	controller_interface.o

include ../../Makefile.rules
