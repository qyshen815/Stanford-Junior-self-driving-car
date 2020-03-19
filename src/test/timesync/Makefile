include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += -ltimesync_interface -lipc_interface $(GLOBAL_LIBS)

MODULE_NAME = TIMESYNC_TEST

SOURCES = test_timesync.cc time_ping.cc

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 

TARGETS = time_ping test_timesync

# rules

time_ping:		time_ping.o

test_timesync:		test_timesync.o

include ../../Makefile.rules
