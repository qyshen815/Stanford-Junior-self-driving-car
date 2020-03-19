include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 

MODULE_NAME = SERIAL

ifeq ($(ARCH), Darwin)
	SOURCES = serial_mac_stub.cpp
else
SOURCES = serial.cpp
endif

PUBLIC_INCLUDES = serial.h
PUBLIC_LIBRARIES = libserial.a
PUBLIC_BINARIES = 

TARGETS = libserial.a

# rules

ifeq ($(ARCH), Darwin)
libserial.a:	serial_mac_stub.o
else
libserial.a:	serial.o
endif

include ../../Makefile.rules
