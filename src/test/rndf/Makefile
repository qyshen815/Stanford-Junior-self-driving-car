include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += -lrndf -lmdf $(GLOBAL_LIBS)

LINK = g++

MODULE_NAME = RNDF_TEST

SOURCES = rndf_test.cpp rndf_read.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 

TARGETS = rndf_test rndf_read

# rules

rndf_test:	rndf_test.o

rndf_read:	rndf_read.o

include ../../Makefile.rules
