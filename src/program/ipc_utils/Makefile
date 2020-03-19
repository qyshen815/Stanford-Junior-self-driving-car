include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 	$(INTERFACE_LIBS) $(GLOBAL_LIBS)

MODULE_NAME = 		IPC_UTILS

SOURCES = 		latency.cc message_monitor.cc

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 	latency message_monitor

TARGETS = 		latency message_monitor

# rules

latency:		latency.o

message_monitor:	message_monitor.o

include ../../Makefile.rules
