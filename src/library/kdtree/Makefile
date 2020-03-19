include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 

MODULE_NAME = KDTREE

SOURCES = kdtree.cpp bucket.cpp

PUBLIC_INCLUDES = kdtree.h bucket.h
PUBLIC_LIBRARIES = libkdtree.a libbucket.a
PUBLIC_BINARIES = 

TARGETS = libkdtree.a libbucket.a

# rules

libkdtree.a:		kdtree.o

libbucket.a:		bucket.o

include ../../Makefile.rules
