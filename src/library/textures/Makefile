include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 

ifeq ($(ARCH), Darwin)
	IFLAGS += -I/usr/X11/include
endif 

MODULE_NAME = TEXTURES

SOURCES = textures.cpp

PUBLIC_INCLUDES = textures.h
PUBLIC_LIBRARIES = libtextures.a
PUBLIC_BINARIES = 

TARGETS = libtextures.a

# rules

libtextures.a:	textures.o

include ../../Makefile.rules
