include ../../Makefile.conf

IFLAGS += $(STANFORD_DRIVING_IFLAGS)

ifneq ($(ARCH), Darwin)
LFLAGS += -Wl,--whole-archive
endif

ifneq ($(ARCH), Darwin)
LFLAGS += -Wl,--no-whole-archive
endif

LFLAGS += $(STANFORD_DRIVING_LFLAGS)

MODULE_NAME = "AW MDF"

SOURCES = aw_CheckpointList.cpp aw_Mission.cpp aw_SpeedLimitList.cpp

PUBLIC_INCLUDES = aw_CheckpointList.h aw_MDFTokens.h aw_Mission.h aw_SpeedLimitList.h
 
PUBLIC_LIBRARIES = libaw_mdf.a
PUBLIC_BINARIES = 

TARGETS = libaw_mdf.a

# rules
libaw_mdf.a: aw_CheckpointList.o aw_Mission.o aw_SpeedLimitList.o



include ../../Makefile.rules
