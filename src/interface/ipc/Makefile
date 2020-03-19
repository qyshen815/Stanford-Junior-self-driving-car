include ../../Makefile.conf

CXXFLAGS += 
IFLAGS += 
LFLAGS +=	-lipc_interface $(GLOBAL_LIBS)
LINK = g++

MODULE_NAME = 		IPC

SOURCES = 		ipc_std_interface.cc multicentral.cc \
			ipc_interface_unittest.cc ipc_callbacks_unittest.cc 

PUBLIC_INCLUDES = 	ipc_std_interface.h ipc_interface.h multicentral.h \
			ipc_callbacks-inl.h
PUBLIC_LIBRARIES = 	libipc_interface.a libmulticentral.a
PUBLIC_BINARIES = 

TARGETS = 		libipc_interface.a libmulticentral.a

TEST_TARGETS =		ipc_interface_unittest ipc_callbacks_unittest

libipc_interface.a:	ipc_std_interface.o 

ipc_interface_unittest:	ipc_interface_unittest.o libipc_interface.a

ipc_callbacks_unittest:	ipc_callbacks_unittest.o

libmulticentral.a:	multicentral.o

include ../../Makefile.rules
