include ../../Makefile.conf

MODULE_NAME = VIDEOOUT

ifeq (1,$(HAVE_VIDEOOUT))
IFLAGS += `pkg-config --cflags libavformat libavutil libswscale` #This doesn't work on 9.10.
LFLAGS += `pkg-config --libs libavformat libavutil libswscale`
IFLAGS += -I/usr/include/libavformat -I/usr/include/libavcodec -I/usr/include/libswscale #For 9.10.

SOURCES = videoout.cpp

PUBLIC_INCLUDES = videoout.h 
PUBLIC_LIBRARIES = libvideoout.a
PUBLIC_BINARIES = 

TARGETS = libvideoout.a 
endif

# rules

libvideoout.a:	videoout.o

include ../../Makefile.rules
