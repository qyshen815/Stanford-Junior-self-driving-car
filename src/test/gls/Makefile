include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += -lgls_interface $(GRAPHICS_LIBS) $(GLOBAL_LIBS)

MODULE_NAME = GLS

SOURCES = gls_display.cpp gls_send.c

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 

TARGETS = gls_display gls_send

# rules

gls_display:	gls_display.o

gls_send:	gls_send.o

include ../../Makefile.rules
