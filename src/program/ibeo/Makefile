include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 	$(INTERFACE_LIBS) $(GLOBAL_LIBS) 
LINK = g++

MODULE_NAME = 		IBEO

SOURCES = 		ibeocore.cpp ibeo.cc ibeo_ipc.cc

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 	ibeo

TARGETS = 		ibeo

# rules

ibeo:		ibeocore.o ibeo.o ibeo_ipc.o 

include ../../Makefile.rules
