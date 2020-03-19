include ../../Makefile.conf

MODULE_NAME = "Stereo Camera"

ifeq (1,$(HAVE_DC1394))
  LFLAGS += $(GLOBAL_LIBS) -lipc_interface -lglobal -llltransform -lipc -ldc1394stereo \
			-lcamera_interface -lshm_wrapper -ldc1394 

  SOURCES 		+= dc1394StereoCam.cpp
  PUBLIC_BINARIES       += dc1394stereocam
  TARGETS               += dc1394stereocam

dc1394stereocam:    	dc1394StereoCam.o
endif

include ../../Makefile.rules
