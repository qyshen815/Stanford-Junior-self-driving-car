include ../../Makefile.conf

IFLAGS += `pkg-config --cflags freetype2`
LFLAGS += `pkg-config --libs freetype2` -lftgl

ifeq ($(ARCH), Darwin)
	IFLAGS += -I/usr/X11/include
endif 

MODULE_NAME = PASSATMODEL

SOURCES = passatgl.cpp passatwagongl.cpp tiregl.cpp velodynegl.cpp passatmodel.cpp lasermodel.cpp

PUBLIC_INCLUDES = passatmodel.h lasermodel.h skybox.h
PUBLIC_LIBRARIES = libpassatmodel.a liblasermodel.a libskybox.a
PUBLIC_BINARIES = 

TARGETS = libpassatmodel.a liblasermodel.a libskybox.a

# rules

liblasermodel.a:	lasermodel.o

libpassatmodel.a:	passatmodel.o passatgl.o passatwagongl.o tiregl.o velodynegl.o

libskybox.a:		skybox.o

include ../../Makefile.rules
