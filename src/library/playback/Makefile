include ../../Makefile.conf

# delete any fields you don't need

CFLAGS +=
IFLAGS += 
LFLAGS += 
LINK = g++

MODULE_NAME = PLAYBACK

SOURCES = camplayer.cpp vlfplayer.cpp llfplayer.cpp data_player.cpp

PUBLIC_INCLUDES = camplayer.h vlfplayer.h llfplayer.h data_player.h
PUBLIC_LIBRARIES = libcamplayer.a libvlfplayer.a libllfplayer.a
PUBLIC_BINARIES = 

TARGETS = 	libcamplayer.a libvlfplayer.a libllfplayer.a 

# rules

libvlfplayer.a:		vlfplayer.o data_player.o

libllfplayer.a:		llfplayer.o data_player.o

libcamplayer.a:		camplayer.o data_player.o

include ../../Makefile.rules
