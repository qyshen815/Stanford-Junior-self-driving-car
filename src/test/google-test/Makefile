include ../../Makefile.conf

CXXFLAGS +=
IFLAGS += 
LFLAGS += -Wl,-rpath=/usr/local/lib
LINK = g++

MODULE_NAME = 		GOOGLE_TEST

SOURCES = 		sample1.cc sample1_unittest.cc \
			sample2.cc sample2_unittest.cc \
			sample3_unittest.cc \
			sample4.cc sample4_unittest.cc \
			sample5_unittest.cc \
			sample6_unittest.cc 

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 

TEST_TARGETS = 		sample1_unittest sample2_unittest \
			sample3_unittest sample4_unittest \
			sample5_unittest sample6_unittest 
EXTRA_CLEAN = 		

TARGETS = 		

# rules

sample1_unittest:	sample1.o sample1_unittest.o

sample2_unittest:	sample2.o sample2_unittest.o

sample3_unittest:	sample3_unittest.o

sample4_unittest:	sample4.o sample4_unittest.o

sample5_unittest:	sample1.o sample5_unittest.o

sample6_unittest:	sample1.o sample6_unittest.o

include ../../Makefile.rules
