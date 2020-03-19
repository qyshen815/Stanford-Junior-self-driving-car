include ../../Makefile.conf

#IFLAGS += -pg #-finstrument-functions -g -O3
#LFLAGS += -pg #-lSaturn

IFLAGS += $(STANFORD_DRIVING_IFLAGS)

ifneq ($(ARCH), Darwin)
LFLAGS += -Wl,--whole-archive
endif

ifneq ($(ARCH), Darwin)
LFLAGS += -Wl,--no-whole-archive
endif

LFLAGS += $(STANFORD_DRIVING_LFLAGS)

LFLAGS += 

MODULE_NAME = CURVE_SMOOTHING

MODULE_COMMENT = 
SOURCES = curveSmoother.cpp clothoid.cpp bezCtxPS.cpp 

PUBLIC_INCLUDES = curveSmoother.h clothoid.h bezCtxPS.h
                  
PUBLIC_LIBRARIES = libcurve_smoothing.a
PUBLIC_BINARIES = 
TARGETS = libcurve_smoothing.a 

# rules
libcurve_smoothing.a : curveSmoother.o clothoid.o bezCtxPS.o
                    
include ../../Makefile.rules
