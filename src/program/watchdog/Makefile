include ../../Makefile.conf

CFLAGS += 
IFLAGS +=  
LFLAGS += $(INTERFACE_LIBS) $(GLOBAL_LIBS) -lipc_interface

MODULE_NAME = WATCHDOG

SOURCES = watchdog.cpp watchdog_notify.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = watchdog watchdog_notify

TARGETS = watchdog watchdog_notify

# rules

watchdog:	watchdog.o

watchdog_notify:	watchdog_notify.o

include ../../Makefile.rules
