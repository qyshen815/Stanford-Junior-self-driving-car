include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 	-lsignal_handler $(INTERFACE_LIBS) $(GLOBAL_LIBS)
LINK = g++

MODULE_NAME = 		RIEGL

SOURCES = 		riegl.cc rieglcore.cc 

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 	riegl

TARGETS = 		riegl

# rules

riegl:			riegl.o rieglcore.o 

include ../../Makefile.rules
