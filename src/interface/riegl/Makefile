include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 

MODULE_NAME = 		RIEGL

SOURCES = 		riegl_interface.cc

PUBLIC_INCLUDES = 	riegl_interface.h riegl_messages.h
PUBLIC_LIBRARIES = 	libriegl_interface.a
PUBLIC_BINARIES = 

TARGETS = 		libriegl_interface.a 

# rules

libriegl_interface.a:	riegl_interface.o

include ../../Makefile.rules
