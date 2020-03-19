include ../../Makefile.conf

CXXFLAGS += -DSIMULATOR_FX -g -frounding-math -DCGAL_DISABLE_ROUNDING_MATH_CHECK
CFLAGS += -frounding-math -DCGAL_DISABLE_ROUNDING_MATH_CHECK
IFLAGS += `pkg-config --cflags freetype2`
IFLAGS += $(QT_INCLUDES) -I/usr/include/libavformat
LFLAGS += -L$(IPPROOT)/sharedlib -L$(IPPROOT)/lib $(INTERFACE_LIBS) -lpointlist -lfacelist -lgrid \
	 -lproject -lrndf $(QT_LIBS) $(GLOBAL_LIBS) -lipc_interface 
	 
ifeq ($(ARCH), Darwin)
LFLAGS += -limage
LFLAGS += -framework OpenGL -framework AGL
else
LFLAGS += $(GRAPHICS_LIBS)
endif

LFLAGS += -law_geometry -law_rndf

LFLAGS += -lcurl -lbil
LFLAGS += -lpassatmodel -lvehiclemodels -limagery -llltransform -lsockutils \
	      	 -lrndfgl -lgui3D -lskybox -lgl_support -ltextures -lftgl \
		 -lglut -lglui -limage
LFLAGS += -law_rndfgl -lrndfgl -lgl_support -ltextures -limage -lIL -lglobal

LFLAGS += $(IPP_LIBS)

ifeq (1,$(HAVE_VIDEOOUT))
	LFLAGS += -lbz2 -lvideoout -lavcodec -lavutil -lavformat -lswscale
endif

MODULE_NAME = PERCEPTION VIEW

QT_SOURCES = 	qt.cpp
SOURCES = 	stop_zones.cpp grid.cpp keyboard.cpp message.cpp utils.cpp \
		graphics.cpp velodyne.cpp ldlrs.cpp lms.cpp \
		main.cpp appl_history.cpp map.cpp display.cpp info.cpp \
		ipc.cpp mesh.cpp help.cpp signs.cpp mouse.cpp trafficlights.cpp

ifeq (1,$(HAVE_QT)) 
 QT_OBJECTS += qt.o qt-moc.o 
 QT_GUI_FILES = qt-moc.cpp 
 EXTRA_CLEAN += $(QT_GUI_FILES) 
endif 

ROS_DEPS = cgal opencv2

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = perception_view

QT_TARGETS = 
TARGETS = perception_view

# rules

perception_view:	stop_zones.o grid.o keyboard.o message.o utils.o \
			graphics.o velodyne.o ldlrs.o lms.o ipc.o \
			appl_history.o map.o display.o info.o main.o mesh.o trafficlights.o\
			signs.o mouse.o help.o $(QT_OBJECTS)

include ../../Makefile.rules
