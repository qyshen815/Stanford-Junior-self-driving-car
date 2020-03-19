include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += -lparam_interface -lipc_interface $(GLOBAL_LIBS)
LINK = g++

MODULE_NAME = PARAM_SERVER

SOURCES = get_transform.cc test_param.cc

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 

TARGETS = get_transform test_param

# rules

get_transform:		get_transform.o

test_param:		test_param.o

include ../../Makefile.rules
