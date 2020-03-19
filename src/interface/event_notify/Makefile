include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 

MODULE_NAME = EVENT_NOTIFY	

SOURCES = 		event_notify_interface.cc

PUBLIC_INCLUDES = 	event_notify_messages.h event_notify_interface.h
PUBLIC_LIBRARIES = 	libevent_notify_interface.a
PUBLIC_BINARIES = 

TARGETS = 		libevent_notify_interface.a

# rules

libevent_notify_interface.a:	event_notify_interface.o

include ../../Makefile.rules
