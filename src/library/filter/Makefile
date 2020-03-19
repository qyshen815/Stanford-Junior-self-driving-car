include ../../Makefile.conf

CXXFLAGS += -O2

IFLAGS += $(STANFORD_DRIVING_IFLAGS)

ifneq ($(ARCH), Darwin)
LFLAGS += -Wl,--whole-archive
endif

ifneq ($(ARCH), Darwin)
LFLAGS += -Wl,--no-whole-archive
endif

LFLAGS += $(STANFORD_DRIVING_LFLAGS)

MODULE_NAME = "Filter \(Kalman / Particle\)"

SOURCES = kalman_filter.cpp gsl.cpp

PUBLIC_INCLUDES = gsl.h kalman_filter.h particle_filter.h 
PUBLIC_LIBRARIES = libfilter.a
PUBLIC_BINARIES = 

TARGETS = libfilter.a

# rules
libfilter.a: kalman_filter.o gsl.o

include ../../Makefile.rules
