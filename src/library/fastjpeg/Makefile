include ../../Makefile.conf

CFLAGS += -w
CFLAGS += -O3# -xB -ip
CXXFLAGS += -O3# -xB -ip
# CFLAGS += -O0 -g
IFLAGS += -I/usr/local/include

# for ipp
LFLAGS += $(IPPLIBS)

ifdef DEBUG_DISPLAY 
LFLAGS += -lhighgui
endif

MODULE_NAME = FASTJPEG

ifeq (1,$(HAVE_IPP))

SOURCES =  fastjpeg.cpp

PUBLIC_INCLUDES = fastjpeg.h
PUBLIC_LIBRARIES = libfastjpeg.a

TARGETS = libfastjpeg.a 
else
TARGETS = no_ipp
endif

# rules

no_ipp: ;@echo " --> Skipping fastjpeg because no IPP detected"
libfastjpeg.a: fastjpeg.o


include ../../Makefile.rules
