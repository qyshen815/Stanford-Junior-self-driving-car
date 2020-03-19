include ../../Makefile.conf

CXXFLAGS += -Wno-strict-aliasing -fno-strict-aliasing
IFLAGS   += -I.
LFLAGS   += 

MODULE_NAME = ANN

SOURCES = ANN.cpp brute.cpp kd_tree.cpp kd_util.cpp kd_split.cpp \
	  kd_dump.cpp kd_search.cpp kd_pr_search.cpp \
	  kd_fix_rad_search.cpp bd_tree.cpp bd_search.cpp \
	  bd_pr_search.cpp bd_fix_rad_search.cpp perf.cpp

PUBLIC_INCLUDES = ANN.h ANNx.h ANNperf.h
PUBLIC_LIBRARIES = libANN-old.a
PUBLIC_BINARIES = 

TARGETS = libANN-old.a

# rules

libANN-old.a:	ANN.o brute.o kd_tree.o kd_util.o kd_split.o \
	  	kd_dump.o kd_search.o kd_pr_search.o \
	  	kd_fix_rad_search.o bd_tree.o bd_search.o \
	  	bd_pr_search.o bd_fix_rad_search.o perf.o

include ../../Makefile.rules

