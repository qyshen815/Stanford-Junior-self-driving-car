include ../../Makefile.conf

IFLAGS += `pkg-config --cflags freetype2`
LFLAGS += -lglobal -lgl_support -lgui3D -lgui2D -ltextures -limage
LFLAGS += -lcamera_interface -lshm_wrapper
LFLAGS += -lglut -lglui
LFLAGS += -lftgl -lIL

LFLAGS += $(IPP_LIBS)

ifeq ($(ARCH), Darwin)
	LFLAGS += -framework OpenGL -framework AGL
else
	LFLAGS += -lGL -lGLU
endif 

ROS_DEPS += opencv2

MODULE_NAME = 		CAMERA_VIEW

SOURCES = 		camera_view.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 	camera_view

TARGETS =	camera_view

camera_view:		camera_view.o

include ../../Makefile.rules
