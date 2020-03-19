include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 

MODULE_NAME = 		HEALTHMON

SOURCES = 		healthmon_interface.cc

PUBLIC_INCLUDES = 	healthmon_messages.h healthmon_interface.h
PUBLIC_LIBRARIES = 	libhealthmon_interface.a
PUBLIC_BINARIES = 

TARGETS = 		libhealthmon_interface.a

# rules

libhealthmon_interface.a:	healthmon_interface.o

include ../../Makefile.rules
