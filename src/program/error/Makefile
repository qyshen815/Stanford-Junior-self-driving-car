include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 		$(INTERFACE_LIBS) $(GLOBAL_LIBS) -lipc_interface
LINK = g++

MODULE_NAME = 		ERROR

SOURCES = 		add_error.cc add_comments.cc

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 	add_error add_comments

TARGETS = 		add_error add_comments

add_error:		add_error.o

add_comments:		add_comments.o

include ../../Makefile.rules
