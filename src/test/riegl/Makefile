include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 	-lipc_interface -lriegl_interface -lparam_interface \
		$(GLOBAL_LIBS)
LINK = g++

MODULE_NAME = 	RIEGL

SOURCES = 	riegl_test.cc

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 

TARGETS = 	riegl_test

# rules

riegl_test:	riegl_test.o

include ../../Makefile.rules
