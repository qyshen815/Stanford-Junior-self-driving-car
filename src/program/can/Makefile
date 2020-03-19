include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 	-lglobal -lsignal_handler -lusbfind $(INTERFACE_LIBS) $(GLOBAL_LIBS)

MODULE_NAME = 		CAN

SOURCES = 		can.cc cancore.cc

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 	can 

TARGETS = 		can

can:			can.o cancore.o 

include ../../Makefile.rules
