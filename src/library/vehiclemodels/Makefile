include ../../Makefile.conf

CFLAGS += 
IFLAGS += 
LFLAGS += 

MODULE_NAME = VEHICLEMODELS

SOURCES = hummer.cpp lotus.cpp porsche.cpp lamborghini.cpp stickered_passatwagongl.cpp

PUBLIC_INCLUDES = vehiclemodels.h
PUBLIC_LIBRARIES = libvehiclemodels.a
PUBLIC_BINARIES = 

TARGETS = libvehiclemodels.a

# rules

libvehiclemodels.a:	vehiclemodels.o lotus.o hummer.o porsche.o lamborghini.o stickered_passatwagongl.o

include ../../Makefile.rules
