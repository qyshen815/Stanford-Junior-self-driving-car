include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 

MODULE_NAME = 		PIDCONTROL

SOURCES = 		pidcontrol_interface.cc

PUBLIC_INCLUDES = 	pidcontrol_messages.h pidcontrol_interface.h
PUBLIC_LIBRARIES = 	libpidcontrol_interface.a
PUBLIC_BINARIES = 

TARGETS = 		libpidcontrol_interface.a

# rules

libpidcontrol_interface.a:	pidcontrol_interface.o

include ../../Makefile.rules
