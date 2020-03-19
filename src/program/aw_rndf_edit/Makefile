include ../../Makefile.conf

MOC    = $(MOC4)
UIC    = $(UIC4)
RCC    = $(QRCC)

CXXFLAGS += -DRNDF_GL

IFLAGS += `pkg-config freetype2 --cflags`

LFLAGS += -llltransform -law_rndf -law_rndfgl -lrndfgl -lgl_support -law_geometry
LFLAGS += -lglobal -lgl_support -llltransform -limagery -limage -ltextures
LFLAGS += -lcurl -lbil -lproj -lsockutils
LFLAGS += -lglut -lboost_program_options-mt -lftgl -lfreetype
LFLAGS += -llltransform -lcurve_smoothing
LFLAGS += `pkg-config freetype2 --libs`

LFLAGS += $(IPPLIBS)

ifeq ($(ARCH), Darwin)
	LFLAGS += -framework OpenGL -framework AGL -framework QtCore -framework QtGui -framework QtOpenGL
else
	IFLAGS += `pkg-config QtCore QtGui QtOpenGL --cflags`
	LFLAGS +=  `pkg-config QtCore QtGui QtOpenGL --libs`
	LFLAGS += -lGL -lGLU
endif 

MODULE_NAME = RNDF_EDIT	
MODULE_COMMENT = 
SOURCES = rndf_edit.cpp glwidget.cpp rndf_edit_gui.cpp RNDFEditGLView.cpp REConsole.cpp conout.cpp #\
#			moc_glwidget.cpp moc_rndf_edit_gui.cpp moc_RNDFEditGLView.cpp moc_REConsole.cpp \
#			REWayPoint.cpp RELane.cpp RESegment.cpp REPerimeterPoint.cpp REPerimeter.cpp \
#			REZone.cpp RESpot.cpp RESpotPoint.cpp tbimages.cpp 
			

UIC_SOURCES = rndf_edit_qtgui.ui #trajectory2Lane_qtgui.ui
MOC_SOURCES = rndf_edit_gui.h glwidget.h RNDFEditGLView.h REConsole.h
RCC_SOURCES = tbimages.qrc

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = aw_rndf_edit
TARGETS = aw_rndf_edit

ROS_DEPS = cgal libdevil opencv2

# rules
aw_rndf_edit: rndf_edit.o glwidget.o rndf_edit_gui.o RNDFEditGLView.o REConsole.o conout.o \
			moc_glwidget.o moc_rndf_edit_gui.o moc_RNDFEditGLView.o moc_REConsole.o \
			REWayPoint.o RELane.o RESegment.o REPerimeterPoint.o REPerimeter.o \
			REZone.o RESpot.o RESpotPoint.o RETrafficLight.o RECrosswalk.o tbimages.o

include ../../Makefile.rules
