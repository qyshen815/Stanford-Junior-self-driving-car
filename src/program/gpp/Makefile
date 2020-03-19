include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += -lgpp -lgrid -lbucket -lrndf $(INTERFACE_LIBS) $(GLOBAL_LIBS)

LINK = g++

MODULE_NAME = GENERAL_PATH_PLANNER

SOURCES = gpp_standalone.cpp thread_example.cpp plot_heuristic.cpp \
	  blank_heuristic.cpp compute_heuristic.cpp 

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = gpp_standalone 

GRAPHICS_TARGETS = plot_heuristic compute_heuristic
TARGETS = gpp_standalone thread_example blank_heuristic

# rules

gpp_standalone:		gpp_standalone.o 

thread_example:		thread_example.o

plot_heuristic:		plot_heuristic.o 

blank_heuristic:	blank_heuristic.o 

compute_heuristic:	compute_heuristic.o

include ../../Makefile.rules
