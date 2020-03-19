include ../../Makefile.conf

IFLAGS += $(STANFORD_DRIVING_IFLAGS)

ifneq ($(ARCH), Darwin)
LFLAGS += -Wl,--whole-archive
endif

ifneq ($(ARCH), Darwin)
LFLAGS += -Wl,--no-whole-archive
endif

LFLAGS += $(STANFORD_DRIVING_LFLAGS)

MODULE_NAME = "AW CHSM Planner"

SOURCES = aw_chsm_planner_interface.cpp

PUBLIC_INCLUDES = aw_chsm_planner_interface.h aw_chsm_planner_messages.h
PUBLIC_LIBRARIES = libaw_chsm_planner_interface.a
PUBLIC_BINARIES = 

TARGETS = libaw_chsm_planner_interface.a

# rules
libaw_chsm_planner_interface.a: aw_chsm_planner_interface.o

include ../../Makefile.rules
