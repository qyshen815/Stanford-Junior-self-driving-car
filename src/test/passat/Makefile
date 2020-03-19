include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += -lpassat_interface -lparam_interface $(GLOBAL_LIBS)

MODULE_NAME = PASSAT

SOURCES = test_turn_signals.c motor_temp.c

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 

TARGETS = test_turn_signals motor_temp

# rules

test_turn_signals:	test_turn_signals.o

motor_temp:		motor_temp.o

include ../../Makefile.rules
