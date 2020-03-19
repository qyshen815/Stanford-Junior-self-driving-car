include ../../Makefile.conf

CFLAGS += -Wno-strict-aliasing -fno-strict-aliasing
IFLAGS   += -I.
LFLAGS   += 

MODULE_NAME = ROTE

SOURCES = rote.c inject.c inject_csi.c

PUBLIC_INCLUDES = rote.h
PUBLIC_LIBRARIES = librote.a
PUBLIC_BINARIES = 

TARGETS = librote.a

# rules

librote.a:	rote.o inject.o inject_csi.o

include ../../Makefile.rules
