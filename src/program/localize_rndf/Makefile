include ../../Makefile.conf

CFLAGS += 
IFLAGS += 
LFLAGS += $(INTERFACE_LIBS) -lfacelist -lpointlist -lrndf $(GLOBAL_LIBS) \
	  -lipc_interface

LINK = g++

MODULE_NAME = LOCALIZE_RNDF

SOURCES = localize_rndf.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = localize_rndf

TARGETS = localize_rndf

# rules

localize_rndf:		localize_rndf.o

include ../../Makefile.rules
