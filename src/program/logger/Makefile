include ../../Makefile.conf

CFLAGS += 
IFLAGS += 
LFLAGS += 		-lb64 -lblf -lasync_writer -lrt -lipc_interface \
			$(INTERFACE_LIBS) $(IMAGEMAGICK_LIBS) $(GLOBAL_LIBS) 
LINK = g++

MODULE_NAME = 		LOGGER

SOURCES = 		dgc_logger.cc param_handling.cc camera_logger.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 	dgc_logger ladybug_logger camera_logger

TARGETS = 		dgc_logger ladybug_logger camera_logger

# rules

dgc_logger:		dgc_logger.o param_handling.o

camera_logger:		camera_logger.o

ladybug_logger:		ladybug_logger.o

include ../../Makefile.rules
