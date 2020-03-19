include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += $(GRAPHICS_LIBS) $(GLOBAL_LIBS)

MODULE_NAME = GUI3D

SOURCES = gui2D_test.c gui3D_test.cpp pointing_test.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 

TARGETS = gui2D_test gui3D_test pointing_test

# rules

gui2D_test:	gui2D_test.o libgui2D.a libgl_support.a

gui3D_test:	gui3D_test.o libgui3D.a libgl_support.a

pointing_test:	pointing_test.o libgui3D.a libgl_support.a

include ../../Makefile.rules
