include ../../Makefile.conf

IFLAGS += $(STANFORD_DRIVING_IFLAGS)

ifneq ($(ARCH), Darwin)
LFLAGS += -Wl,--whole-archive
endif

ifneq ($(ARCH), Darwin)
LFLAGS += -Wl,--no-whole-archive
endif

LFLAGS += $(STANFORD_DRIVING_LFLAGS)

MODULE_NAME = "AW Navigator"

SOURCES = aw_navigator_interface.cpp

PUBLIC_INCLUDES = aw_navigator_interface.h aw_navigator_messages.h
PUBLIC_LIBRARIES = libaw_navigator_interface.a
PUBLIC_BINARIES = 

TARGETS = libaw_navigator_interface.a

# rules
libaw_navigator_interface.a: aw_navigator_interface.o

include ../../Makefile.rules
