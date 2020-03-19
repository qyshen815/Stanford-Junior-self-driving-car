include ../../Makefile.conf


MODULE_NAME = 		GLOBAL

SOURCES = 		global.cpp transform.cpp dgc_stdio.cpp dgc_curses.cpp signal_handler.cpp \
			scaledTime.cpp pose.cpp poseQueue.cpp

PUBLIC_INCLUDES = 	global.h roadrunner.h dgc_stdio.h \
		  	touareg_constants.h passat_constants.h transform.h \
			dgc_curses.h signal_handler.h vlrException.h scaledTime.h \
			pose.h poseQueue.h

ifeq ($(ARCH), Darwin)
SOURCES		+= macAddOns.cpp
PUBLIC_INCLUDES += macAddOns.h
endif

PUBLIC_LIBRARIES = 	libglobal.a libdgc_curses.a libsignal_handler.a
PUBLIC_BINARIES = 

TARGETS = 		libglobal.a libdgc_curses.a libsignal_handler.a

# rules

ifeq ($(ARCH), Darwin)
libglobal.a:		global.o dgc_stdio.o transform.o scaledTime.o pose.o poseQueue.o macAddOns.o
else
libglobal.a:		global.o dgc_stdio.o transform.o scaledTime.o pose.o poseQueue.o
endif
libsignal_handler.a:	signal_handler.o

libdgc_curses.a:	dgc_curses.o

include ../../Makefile.rules
