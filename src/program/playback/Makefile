include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += -lsignal_handler -lvlfplayer -lllfplayer -lcamplayer -lblf \
	  -lasync_writer -lrt -ldgc_curses \
	  -lb64 $(INTERFACE_LIBS) $(GLOBAL_LIBS) -lncurses
LINK = g++

MODULE_NAME = PLAYBACK

SOURCES = dgc_playback.cpp extract_files_from_log.cc playback_server.cc \
	  ladybug_playback_server.cc ladybug_playback_standalone.cpp \
	  velodyne_playback_server.cc velodyne_playback_standalone.cpp \
	  camera_playback_server.cc camera_playback_standalone.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = dgc_playback extract_files_from_log \
		  ladybug_playback_standalone velodyne_playback_standalone \
		  camera_playback_standalone 

TARGETS = 	dgc_playback extract_files_from_log \
		ladybug_playback_standalone velodyne_playback_standalone \
		camera_playback_standalone

# rules

dgc_playback:			playback_server.o velodyne_playback_server.o \
				ladybug_playback_server.o dgc_playback.o

extract_files_from_log:		extract_files_from_log.o

velodyne_playback_standalone:	velodyne_playback_server.o \
				velodyne_playback_standalone.o

ladybug_playback_standalone:	ladybug_playback_server.o \
				ladybug_playback_standalone.o

camera_playback_standalone:	camera_playback_server.o \
				camera_playback_standalone.o

include ../../Makefile.rules
