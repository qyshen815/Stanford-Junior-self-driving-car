#COMPILER_ICC=1

include ../../Makefile.conf

MODULE_NAME = TRACKER

SOURCES = vectors.cpp timer.cpp misc.cpp tree.cpp tracker.cpp \
	sampling.cpp trackcore.cpp rndf_lookup.cpp angle_grid.cpp \
	projected_scan.cpp libtracker.cpp 

PUBLIC_INCLUDES = libtracker.h
PUBLIC_LIBRARIES = libtracker.a
PUBLIC_BINARIES =

TARGETS = libtracker.a

# rules

libtracker.a: vectors.o timer.o tree.o \
	libtracker.o trackcore.o sampling.o rndf_lookup.o \
	angle_grid.o tracker.o projected_scan.o misc.o 


include ../../Makefile.rules
