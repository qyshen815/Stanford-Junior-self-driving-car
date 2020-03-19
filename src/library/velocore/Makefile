include ../../Makefile.conf

CFLAGS += 	 -DCHECK_INSIDE_CAR
CXXFLAGS +=  -fno-strict-aliasing -DCHECK_INSIDE_CAR
IFLAGS += 
LFLAGS += 

MODULE_NAME = 		VELOCORE

SOURCES = 		velocore.cpp veloconfig.cpp velo_support.cpp

PUBLIC_INCLUDES = 	velocore.h velo_support.h 
PUBLIC_LIBRARIES = 	libvelocore.a libvelo_support.a 
PUBLIC_BINARIES = 

TARGETS = 		libvelocore.a libvelo_support.a 

# rules

libvelocore.a:		velocore.o veloconfig.o 

libvelo_support.a:	velo_support.o

include ../../Makefile.rules
