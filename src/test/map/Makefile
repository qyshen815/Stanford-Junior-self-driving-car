include ../../Makefile.conf

CFLAGS +=
IFLAGS += 

LFLAGS += -lgrid -lapplanix_interface -llocalize_interface \
	  -lperception_interface -libeo_interface -lparam_interface \
	  $(GLOBAL_LIBS)

LINK = g++

MODULE_NAME = SIMPLEMAPPER

SOURCES = simplemapper.c simplemapper_ipc.c

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 

TARGETS = simplemapper 

# rules

simplemapper:	simplemapper.o simplemapper_ipc.o

include ../../Makefile.rules
