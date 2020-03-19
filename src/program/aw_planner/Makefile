include ../../Makefile.conf

MODULE_NAME = AW PLANNER 

CXXFLAGS += -DRNDF_GL
IFLAGS += `pkg-config freetype2 --cflags`

ifeq (1, $(HAVE_PROBT))
# probt stuff
IFLAGS += -I../../ext/probt/include
LFLAGS += -L../../ext/probt/lib -lspl 
LFLAGS += -lboost_serialization-gcc42-mt-1_34_1 -lboost_thread-gcc42-mt-1_34_1
LFLAGS += -ltcc
endif

LFLAGS += -lipc_interface -lparam_interface -lpassat_interface -lperception_interface
LFLAGS += -lglobal -lipc -llltransform -llogio -ltextures -limage -lbucket
LFLAGS += -limagery -lsockutils -lcurl -lbil -llltransform -lproj -lproject
LFLAGS += -ltrajectory -lsignal_handler -lpoly_traj 
LFLAGS += -law_chsm_planner -law_topology -law_mdf -law_rndf -law_rndfgl -lrndfgl
LFLAGS += -law_geometry -law_timestamp
LFLAGS += -lglobal
LFLAGS += -lcar_list

ifeq (1,$(HAVE_IPP))

	ifeq ($(ARCH), Darwin)
		LFLAGS += -framework OpenGL -framework AGL
	else
		LFLAGS += -lGL -lGLU
	endif 

	LFLAGS += $(IPPLIBS)
	LFLAGS += -lz -lIL
	LFLAGS += -lglut -lglui
	LFLAGS += -lpoly_traj -lcurve_smoothing -lgui3D -lpassatmodel -lvehiclemodels -lgl_support -ltextures -limage
	LFLAGS += `pkg-config freetype2 --libs` -lftgl

	SOURCES =	aw_planner.cpp aw_planner_gui.cpp makeCircleRNDF.cpp baseDemo.cpp circle_demo.cpp static_map_demo.cpp \
				fakeObstacleTracker.cpp fake_car.cpp 

	PUBLIC_INCLUDES = 
	PUBLIC_LIBRARIES = 
	PUBLIC_BINARIES = aw_planner

	TARGETS = aw_planner fake_car 
else
	TARGETS = no_ipp
endif


ROS_DEPS = cgal google-perftools opencv2 libdevil

PROFILE = 1
ifeq (1,$(PROFILE))
LFLAGS += -lprofiler
endif

# rules

no_ipp: ;@echo " --> Not building planner because no IPP installed"
aw_planner:	aw_planner.o aw_planner_gui.o baseDemo.o circle_demo.o static_map_demo.o fakeObstacleTracker.o
makeCircleRNDF: makeCircleRNDF.o 
fake_car: fake_car.o

include ../../Makefile.rules
