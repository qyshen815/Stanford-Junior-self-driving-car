include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 

MODULE_NAME = 		HCI

SOURCES = 		hci_interface.cc

PUBLIC_INCLUDES = 	hci_messages.h hci_interface.h
PUBLIC_LIBRARIES = 	libhci_interface.a
PUBLIC_BINARIES = 

TARGETS = 		libhci_interface.a

# rules

libhci_interface.a:	hci_interface.o

include ../../Makefile.rules
