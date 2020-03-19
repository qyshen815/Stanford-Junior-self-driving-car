include ../../Makefile.conf

CFLAGS +=
IFLAGS +=	`pkg-config --cflags glib-2.0 libgtop-2.0`
LFLAGS += 	`pkg-config --libs glib-2.0 libgtop-2.0` \
	   	-lusbfind $(INTERFACE_LIBS) $(GLOBAL_LIBS)

LINK = g++

MODULE_NAME = 		HEALTHMON

SOURCES = 		healthmon.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 	healthmon

TARGETS = 		healthmon 

healthmon:		healthmon.o

include ../../Makefile.rules
