include ../../Makefile.conf

IFLAGS += $(STANFORD_DRIVING_IFLAGS)

ifneq ($(ARCH), Darwin)
LFLAGS += -Wl,--whole-archive
endif

ifneq ($(ARCH), Darwin)
LFLAGS += -Wl,--no-whole-archive
endif

LFLAGS += $(STANFORD_DRIVING_LFLAGS)

MODULE_NAME = "Trajectory Points"

SOURCES = trajectory_points_interface.cpp

PUBLIC_INCLUDES = trajectory_points_interface.h trajectory_points_messages.h
PUBLIC_LIBRARIES = libtrajectory_points_interface.a 
PUBLIC_BINARIES = 

TARGETS = libtrajectory_points_interface.a

# rules
libtrajectory_points_interface.a: trajectory_points_interface.o

include ../../Makefile.rules
