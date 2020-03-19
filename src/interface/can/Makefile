include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 

MODULE_NAME = 		CAN

SOURCES = 		can_interface.cc

PUBLIC_INCLUDES = 	can_interface.h can_messages.h
PUBLIC_LIBRARIES = 	libcan_interface.a
PUBLIC_BINARIES = 

TARGETS = 		libcan_interface.a

# rules

libcan_interface.a:	can_interface.o

include ../../Makefile.rules
