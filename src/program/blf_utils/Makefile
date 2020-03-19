include ../../Makefile.conf

LFLAGS += $(INTERFACE_LIBS) -lblf -lasync_writer -lrt $(GLOBAL_LIBS) 
LINK = g++

MODULE_NAME = BLF-TOOLS

SOURCES          = blf-index.cpp blf-reduce.cpp blf-status.cpp
PUBLIC_INCLUDES  = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES  = blf-index blf-reduce
TARGETS          = blf-index blf-reduce blf-status

# rules

blf-index:	blf-index.o

blf-reduce:	blf-reduce.o

blf-status:	blf-status.o

include ../../Makefile.rules
