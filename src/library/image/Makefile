include ../../Makefile.conf

CFLAGS += 	
IFLAGS += -I$(IPPROOT)/include/
LFLAGS +=

MODULE_NAME = IMAGE

SOURCES = image.cpp

ifeq (1,$(HAVE_IPP))
  SOURCES += imageipp.cpp
endif

ROS_DEPS = opencv2

PUBLIC_INCLUDES = image.h 
PUBLIC_LIBRARIES = libimage.a
PUBLIC_BINARIES = 

TARGETS = libimage.a

# rules

OBJ = image.o 

ifeq (1,$(HAVE_IPP))
  OBJ += imageipp.o
endif

libimage.a:	$(OBJ)

include ../../Makefile.rules
