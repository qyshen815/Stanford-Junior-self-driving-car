include ../../Makefile.conf

IFLAGS += `pkg-config --cflags freetype2`

MODULE_NAME = 		IMAGERY

SOURCES = 		imagery.cc imagery_proj.cc texturecache.cpp \
			imagery_tcp.cc

PUBLIC_INCLUDES = 	imagery.h imagery_proj.h
PUBLIC_LIBRARIES = 	libimagery.a
PUBLIC_BINARIES = 

TARGETS = 		libimagery.a 

# rules

libimagery.a:		imagery.o imagery_proj.o texturecache.o imagery_tcp.o

include ../../Makefile.rules
