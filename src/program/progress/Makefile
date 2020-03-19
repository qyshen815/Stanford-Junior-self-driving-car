include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += $(INTERFACE_LIBS) $(GLOBAL_LIBS) -lipc_interface
LINK = g++

MODULE_NAME = PROGRESS

SOURCES = progress.cc odometer.cc

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = progress odometer

TARGETS = progress odometer

# rules

progress:	progress.o

odometer:	odometer.o

include ../../Makefile.rules
