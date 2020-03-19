include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 	-lsignal_handler $(INTERFACE_LIBS) $(GLOBAL_LIBS)
LINK = g++

MODULE_NAME = 		PARAM_SERVER

SOURCES = 		param_server.cc param_server_main.cc paramread.cc

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 	param_server install_extra_params

TARGETS = 		param_server install_extra_params

# rules

param_server:		param_server.o paramread.o param_server_main.o

install_extra_params:	install_extra_params.o paramread.o

include ../../Makefile.rules
