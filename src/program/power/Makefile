include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 	-lipc_interface -lusbfind $(INTERFACE_LIBS) $(GLOBAL_LIBS)
LINK = g++

MODULE_NAME = 		POWER

SOURCES = 		power.cc power_ipc.cc

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 	power

TARGETS = 		power

# rules

power:		power.o power_ipc.o 

include ../../Makefile.rules
