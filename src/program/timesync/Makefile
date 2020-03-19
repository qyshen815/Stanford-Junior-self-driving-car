include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 		-lipc_interface $(INTERFACE_LIBS) $(GLOBAL_LIBS) 
LINK = g++

MODULE_NAME = 		TIMESYNC

SOURCES = 		udp.cc timesync_server.cc timesync_client.cc \
			print_syncs.cc

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 

TARGETS = 		timesync_server timesync_client print_syncs

# rules

timesync_server:	timesync_server.o udp.o

timesync_client:	timesync_client.o udp.o

print_syncs:		print_syncs.o

include ../../Makefile.rules
