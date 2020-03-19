include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += 

LINK = g++

MODULE_NAME = GENERAL_PATH_PLANNER

SOURCES = gpp.cpp gpp_path.cpp smoother.cpp gpp_thread.cpp interp.cpp \
	  message_queue.cpp


PUBLIC_INCLUDES = gpp_thread.h gpp.h gpp_path.h smoother.h interp.h \
		  heuristic.h message_queue.h
PUBLIC_LIBRARIES = libgpp.a
PUBLIC_BINARIES = 

TARGETS = libgpp.a

# rules

libgpp.a:		smoother.o gpp.o gpp_path.o gpp_thread.o interp.o \
			heuristic.o message_queue.o

include ../../Makefile.rules
