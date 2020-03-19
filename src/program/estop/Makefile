include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 		-lusbfind $(INTERFACE_LIBS) $(GLOBAL_LIBS) \
			-lipc_interface
LINK = g++

MODULE_NAME = 		ESTOP

SOURCES = 		fake_estop.cc estop.cc broadcaster.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 	fake_estop estop

TARGETS = 		fake_estop estop broadcaster

# rules

fake_estop:		fake_estop.o

estop:			estop.o

broadcaster:		broadcaster.o

include ../../Makefile.rules
