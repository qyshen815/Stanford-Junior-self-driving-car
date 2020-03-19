include ../../Makefile.conf

MODULE_NAME = "CAMERA"

ifeq (1,$(HAVE_DC1394))
  LFLAGS += 	$(GLOBAL_LIBS) -lipc_interface \
		-lglobal -llltransform -lipc -ldc1394camera \
		-lcamera_interface -lshm_wrapper -ldc1394 

  LINK = g++

  SOURCES 		+= dc1394cam.cpp
  PUBLIC_BINARIES       += dc1394cam
  TARGETS               += dc1394cam
endif

dc1394cam:    	dc1394cam.o

include ../../Makefile.rules
