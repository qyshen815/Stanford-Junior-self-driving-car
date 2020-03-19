include ../../Makefile.conf

CFLAGS += -Wno-error
IFLAGS += 
LFLAGS += 

MODULE_NAME = JOYSTICK

ifeq ($(ARCH), Darwin)
SOURCES = joystick_mac_stub.cpp
else
SOURCES = joystick.cpp
endif

PUBLIC_INCLUDES = joystick.h
PUBLIC_LIBRARIES = libjoystick.a
PUBLIC_BINARIES = 

TARGETS = libjoystick.a

# rules

ifeq ($(ARCH), Darwin)
libjoystick.a:		joystick_mac_stub.o
else
libjoystick.a:		joystick.o
endif

include ../../Makefile.rules
