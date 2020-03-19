include ../../Makefile.conf

MODULE_NAME = "BLF Binary Log File Test"
MODULE_COMMENT = "Tests for binary log files"

CFLAGS +=
IFLAGS += 
LFLAGS += -lblf -lasync_writer -lrt -lstdc++ $(GLOBAL_LIBS)

SOURCES = 		blf-write.cpp blf-read.cpp blf_leapfrog.cpp


PUBLIC_INCLUDES  = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES  = 

TARGETS = 		blf-write blf-read blf_leapfrog

blf-write:		blf-write.o

blf-read:		blf-read.o

blf_leapfrog:		blf_leapfrog.o

# rules

include ../../Makefile.rules
