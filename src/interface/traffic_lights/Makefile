include ../../Makefile.conf

IFLAGS += $(STANFORD_DRIVING_IFLAGS)

ifneq ($(ARCH), Darwin)
LFLAGS += -Wl,--whole-archive
endif

ifneq ($(ARCH), Darwin)
LFLAGS += -Wl,--no-whole-archive
endif

LFLAGS += $(STANFORD_DRIVING_LFLAGS)

MODULE_NAME = "Taffic Lights"

SOURCES = 

PUBLIC_INCLUDES = traffic_light_interface.h traffic_light_messages.h
PUBLIC_LIBRARIES = # traffic_light_interface.a
PUBLIC_BINARIES = 

TARGETS = # traffic_light_interface.a

# rules
#traffic_light_interface.a:

include ../../Makefile.rules