DEBUG = 1

include ../../Makefile.conf

CFLAGS += -DDELAY_WARNING
CFLAGS += 
IFLAGS += 
LFLAGS += -lusbfind $(INTERFACE_LIBS) $(GLOBAL_LIBS) -lipc_interface
LINK = g++

MODULE_NAME = 		PASSAT

SOURCES = 		passat.cc passatcore.cc passat_ipc.cc passat_fsm.cc \
			passat_test.cc set_parking_brake.cc

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 	passat passat_test set_parking_brake

TARGETS = 		passat passat_test set_parking_brake

passat:			passat.o passatcore.o passat_ipc.o passat_fsm.o

passat_test:		passat_test.o passatcore.o passat_ipc.o

set_parking_brake:	set_parking_brake.o passatcore.o passat_ipc.o

include ../../Makefile.rules
