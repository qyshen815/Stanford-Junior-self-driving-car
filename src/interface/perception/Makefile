include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 

MODULE_NAME = 		PERCEPTION

SOURCES = 		perception_interface.cc

PUBLIC_INCLUDES = 	perception_messages.h perception_interface.h obstacle_types.h
PUBLIC_LIBRARIES = 	libperception_interface.a
PUBLIC_BINARIES = 

TARGETS = 		libperception_interface.a

# rules

libperception_interface.a:	perception_interface.o

include ../../Makefile.rules
