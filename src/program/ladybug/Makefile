include ../../Makefile.conf

have_libladybug  := $(wildcard ../../../../../lib/libladybug_video.a) 
#have_libdc1394v1 := $(wildcard /usr/include/dc1394/dc1394_control.h /usr/local/include/dc1394/dc1394_control.h)

# GENERAL

LFLAGS += $(INTERFACE_LIBS) -lblf -lasync_writer -lrt $(GLOBAL_LIBS) -lipc_interface -lIL -lftgl
LINK = g++
IFLAGS += -I/usr/include/freetype2

MODULE_NAME = LADYBUG

ROS_DEPS = opencv2

SOURCES          = 
PUBLIC_INCLUDES  = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES  = 
TARGETS          = 

ROS_DEPS = opencv2
# VIEWER

ifeq (1,$(HAVE_IPP)) 
  LDB_LFLAGS = -lladybug_video -lladybug_frameparser -lCg -lCgGL \
	       -lfastjpeg $(IPPLIBS)

  ifeq (1,$(HAVE_QT))
    QT_SOURCES   = qt.cpp
    QT_OBJECTS  += qt.o qt-moc.o
    QT_GUI_FILES = qt-moc.cpp
    IFLAGS      += $(QT_INCLUDES)
    llf_view_LFLAGS  += $(LDB_LFLAGS) $(QT_LIBS) 
    SOURCES              += llf_view.cpp
    PUBLIC_BINARIES      += llf_view
    GRAPHICS_TARGETS     += llf_view
    EXTRA_CLEAN += $(QT_GUI_FILES)
  endif

  ladybug_view_LFLAGS  += $(LDB_LFLAGS)
  SOURCES              += ladybug_view.cpp 
  PUBLIC_BINARIES      += ladybug_view 
  GRAPHICS_TARGETS     += ladybug_view

	ladybug_playback_test_LFLAGS = -lladybug_video -limage -lblf -lasync_writer -lglobal -lfastjpeg -lladybug_interface $(IPPLIBS) -ldc1394
	SOURCES							 += ladybug_playback_test.cpp
	TARGETS						+= ladybug_playback_test	
	ladybug_playback_test_LFLAGS += -lladybug_frameparser
else
  TARGETS          += noviewer
endif

# FIREWIRE PRG

ifeq (1,$(HAVE_DC1394))
  ladybug_LFLAGS       += -ldc1394
  SOURCES              += ladybug_main.cc ladybug_dc1394_v2.cc
  PUBLIC_BINARIES      += ladybug
  TARGETS              += ladybug
  DC1394_OBJ           := ladybug_dc1394_v2.o
else 
  #ifneq ($(strip $(have_libdc1394v1)),) 
  #  ladybug_LFLAGS       += -ldc1394
  #  SOURCES              += ladybug_main.cc ladybug_dc1394_v1.cc
  #  DC1394_OBJ           := ladybug_dc1394_v1.o
  #  PUBLIC_BINARIES      += ladybug
  #  TARGETS              += ladybug
  #else
    TARGETS              += noladybug
  #endif
endif

# rules

noviewer:		;@echo " --> Skip ladybug_view/llf_view, no libladybug"

noladybug:		;@echo " --> Skip ladybug, no libdc1394"

ladybug_playback_test:  ladybug_playback_test.o

ladybug_view:		ladybug_view.o

ladybug:		$(DC1394_OBJ) ladybug_main.o

ifeq (1,$(HAVE_QT))
llf_view:		llf_view.o $(QT_OBJECTS) 
endif

llf-reduce:		llf-reduce.o

llf-index:		llf-index.o

include ../../Makefile.rules
