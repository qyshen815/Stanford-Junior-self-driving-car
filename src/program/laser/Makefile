include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 	-lusbfind -lsignal_handler $(INTERFACE_LIBS) $(GLOBAL_LIBS)
LINK = g++

MODULE_NAME = 		LASER

SOURCES = 		sick.cc laser.cc laser_main.cc

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 	laser 

TARGETS = 		laser 

laser:			laser.o sick.o laser_main.o

include ../../Makefile.rules
