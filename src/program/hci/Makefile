include ../../Makefile.conf

CFLAGS +=
IFLAGS += `pkg-config sdl --cflags`
LFLAGS += `pkg-config sdl --libs` -lSDL_mixer \
	   $(INTERFACE_LIBS) -lipc_interface $(GLOBAL_LIBS) 

LINK = g++

MODULE_NAME = HCI

SOURCES = hci.cpp hci_test.cpp 

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = hci

TARGETS = hci hci_test 

# rules

hci: 		hci.o

hci_test: 	hci_test.o

include ../../Makefile.rules
