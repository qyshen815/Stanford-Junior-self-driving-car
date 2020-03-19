include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += -ljoystick $(INTERFACE_LIBS) $(GLOBAL_LIBS)

MODULE_NAME = JOYROADRUNNER

SOURCES = joyroadrunner.cc keyroadrunner.cc steering_command.cc \
	  throttle_command.cc brake_command.cc rev_engine.cc \
	  steering_wheel.cc

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = joyroadrunner steering_command throttle_command \
		  brake_command rev_engine steering_wheel

TARGETS = steering_command throttle_command brake_command \
	  rev_engine joyroadrunner steering_wheel

# rules

joyroadrunner:		joyroadrunner.o

keyroadrunner:		keyroadrunner.o

brake_command:		brake_command.o

throttle_command:	throttle_command.o

steering_command:	steering_command.o

rev_engine:		rev_engine.o

steering_wheel:		steering_wheel.o

include ../../Makefile.rules
