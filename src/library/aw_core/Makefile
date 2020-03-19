include ../../Makefile.conf

IFLAGS += $(STANFORD_DRIVING_IFLAGS)

ifneq ($(ARCH), Darwin)
LFLAGS += -Wl,--whole-archive
endif

ifneq ($(ARCH), Darwin)
LFLAGS += -Wl,--no-whole-archive
endif

LFLAGS += $(STANFORD_DRIVING_LFLAGS)

MODULE_NAME = "AW Kogmo Core"

SOURCES = #aw_kogmo.cpp aw_kogmo_coordinates.cpp aw_kogmo_transform.cpp aw_kogmo_params.cpp

PUBLIC_INCLUDES = aw_kogmo_math.h # aw_kogmo.h aw_kogmo_coordinates.h aw_kogmo_transform.h \
                  aw_kogmo_constants.h aw_kogmo_vehicle_constants.h aw_kogmo_params.h 
PUBLIC_LIBRARIES = #libaw_kogmo_core.a
PUBLIC_BINARIES = 

TARGETS = #libaw_kogmo_core.a

# rules
#libaw_kogmo_core.a: aw_kogmo.o aw_kogmo_coordinates.o aw_kogmo_transform.o aw_kogmo_params.o



include ../../Makefile.rules
