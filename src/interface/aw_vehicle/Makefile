include ../../Makefile.conf

IFLAGS += $(STANFORD_DRIVING_IFLAGS)

ifneq ($(ARCH), Darwin)
LFLAGS += -Wl,--whole-archive
endif

ifneq ($(ARCH), Darwin)
LFLAGS += -Wl,--no-whole-archive
endif

LFLAGS += $(STANFORD_DRIVING_LFLAGS)

MODULE_NAME = "AW Vehicle"

SOURCES = aw_vehicle_interface.cpp

PUBLIC_INCLUDES = aw_vehicle_interface.h aw_vehicle_messages.h
PUBLIC_LIBRARIES = libaw_vehicle_interface.a
PUBLIC_BINARIES = 

TARGETS = libaw_vehicle_interface.a

# rules
libaw_vehicle_interface.a: aw_vehicle_interface.o

include ../../Makefile.rules
