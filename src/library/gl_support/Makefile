include ../../Makefile.conf

CFLAGS += 
IFLAGS += `pkg-config freetype2 --cflags`
LFLAGS += `pkg-config freetype2 --libs`

MODULE_NAME = 		GL_SUPPORT

SOURCES = 		gl_support.cc gui2D.cc gui3D.cpp trafficlight.cpp FontRenderer.cpp

PUBLIC_INCLUDES = 	gl_support.h gui2D.h gui3D.h rndfgl.h qgui3D.h FontRenderer.h
PUBLIC_LIBRARIES = 	libgl_support.a libgui2D.a libgui3D.a \
			librndfgl.a libqgui3D.a
PUBLIC_BINARIES = 

QT_SOURCES = 		qgui3D.cpp
QT_TARGETS = 		libqgui3D.a 

TARGETS = 		libgl_support.a libgui2D.a libgui3D.a librndfgl.a

# rules

libgl_support.a:	gl_support.o FontRenderer.o

librndfgl.a:		rndfgl.o trafficlight.o

libgui2D.a:		gui2D.o

libgui3D.a:		gui3D.o

libqgui3D.a:		qgui3D.o

include ../../Makefile.rules
