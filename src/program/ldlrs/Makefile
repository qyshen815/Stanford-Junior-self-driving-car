include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 	-lipc_interface $(INTERFACE_LIBS) $(GLOBAL_LIBS)
LINK = g++

MODULE_NAME = 		LDLRS

SOURCES = 		ldlrs.cc ldlrscore.cpp ldlrs_ipc.cc 

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 	ldlrs

TARGETS = 		ldlrs

# rules

ldlrs:		ldlrs.o ldlrscore.o ldlrs_ipc.o 

include ../../Makefile.rules
