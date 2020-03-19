include ../../Makefile.conf

IFLAGS += $(STANFORD_DRIVING_IFLAGS)

ifneq ($(ARCH), Darwin)
LFLAGS += -Wl,--whole-archive
endif

ifneq ($(ARCH), Darwin)
LFLAGS += -Wl,--no-whole-archive
endif

LFLAGS += $(STANFORD_DRIVING_LFLAGS)

MODULE_NAME = "AW Timestamp"

SOURCES = aw_timestamp.cpp

PUBLIC_INCLUDES = aw_timestamp.h aw_timestamp.hpp
PUBLIC_LIBRARIES = libaw_timestamp.a
PUBLIC_BINARIES = 

TARGETS = libaw_timestamp.a

# rules
libaw_timestamp.a: aw_timestamp.o

include ../../Makefile.rules
