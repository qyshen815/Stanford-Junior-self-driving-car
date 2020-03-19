include ../../Makefile.conf

CFLAGS += 
IFLAGS += 
LFLAGS += 		-lusbfind $(INTERFACE_LIBS) $(GLOBAL_LIBS) \
			-lipc_interface
LINK = g++

MODULE_NAME = 		CAN

SOURCES = kvasercan.cpp usbcan.cpp can.cpp candump.cpp

PUBLIC_INCLUDES = can.h kvasercan.h
PUBLIC_LIBRARIES = libcan.a
PUBLIC_BINARIES = 

TARGETS = libcan.a		 

# rules

libcan.a: kvasercan.o can.o

include ../../Makefile.rules
