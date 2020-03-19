include ../../Makefile.conf

CFLAGS += 
IFLAGS += 
LFLAGS += 

MODULE_NAME = 		PARAM_SERVER

SOURCES = 		commandline.cc param_interface.cc 

PUBLIC_INCLUDES = 	commandline.h param_interface.h param_messages.h 
PUBLIC_LIBRARIES = 	libparam_interface.a
PUBLIC_BINARIES = 

TARGETS = 		libparam_interface.a

# rules

libparam_interface.a:	commandline.o param_interface.o 

include ../../Makefile.rules
