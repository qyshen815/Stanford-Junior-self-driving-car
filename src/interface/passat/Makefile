include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 

MODULE_NAME = 		PASSAT

SOURCES = 		passat_interface.cc

PUBLIC_INCLUDES = 	passat_interface.h passat_messages.h passat_fsm_states.h
PUBLIC_LIBRARIES = 	libpassat_interface.a
PUBLIC_BINARIES = 

TARGETS = libpassat_interface.a

# rules

libpassat_interface.a:	passat_interface.o

include ../../Makefile.rules
