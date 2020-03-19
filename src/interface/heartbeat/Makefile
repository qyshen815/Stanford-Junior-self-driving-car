include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 

MODULE_NAME = 		HEARTBEAT

SOURCES = 		heartbeat_interface.cc

PUBLIC_INCLUDES = 	heartbeat_messages.h heartbeat_interface.h
PUBLIC_LIBRARIES = 	libheartbeat_interface.a
PUBLIC_BINARIES = 

TARGETS = 		libheartbeat_interface.a

# rules

libheartbeat_interface.a:	heartbeat_interface.o

include ../../Makefile.rules
