include ../../Makefile.conf

MODULE_NAME = "SHM Test"
MODULE_COMMENT = "Tests for the SHM interface"

CFLAGS +=
IFLAGS += 
LFLAGS += -lshm_wrapper $(GLOBAL_LIBS)
LINK    = g++

SOURCES = shm-write.cpp shm-read.cpp


PUBLIC_INCLUDES  = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES  = 
MAN_PAGES =

TARGETS = shm-write shm-read

shm-write:	shm-write.o

shm-read:	shm-read.o

# rules

include ../../Makefile.rules
