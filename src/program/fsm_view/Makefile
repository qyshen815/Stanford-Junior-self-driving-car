include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += -lipc_interface $(INTERFACE_LIBS) $(GLOBAL_LIBS)
LINK = g++

MODULE_NAME = FSM_VIEW

SOURCES = fsm_view.cpp fsmdraw.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = fsm_view

GRAPHICS_TARGETS = fsm_view
TARGETS = 

# rules

fsm_view:	fsm_view.o fsmdraw.o

include ../../Makefile.rules
