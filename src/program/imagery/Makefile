include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += $(GLOBAL_LIBS) -lcurl

MODULE_NAME = IMAGERY

SOURCES = imagery_server.cc download_imagery.cpp terraserver.cpp create_boundary.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 

GRAPHICS_TARGETS = imagery_server download_imagery
TARGETS = create_boundary #old_download_imagery

# rules

imagery_server:		imagery_server.o 

download_imagery:	download_imagery.o terraserver.o

create_boundary:	create_boundary.o

#old_download_imagery:	old_download_imagery.o terraserver.o

include ../../Makefile.rules
