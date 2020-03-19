include ../../Makefile.conf

CFLAGS +=
IFLAGS += `pkg-config --cflags freetype2`
LFLAGS += -ltrajectory -lrndf -lmdf $(INTERFACE_LIBS) $(GLOBAL_LIBS) \
	  -lipc_interface 
LINK = g++

MODULE_NAME = FAKE_PLANNER

SOURCES = fake_planner.cc fp.cc create_base_trajectory.cpp smoother.cpp \
	  spline.cpp bt.cpp dp.cpp \
	  planner_data.cpp fake_localize.cc back_and_forth.cc \
	  fake_planner_fb.cc

PUBLIC_INCLUDES = fp.h
PUBLIC_LIBRARIES = libfp.a
PUBLIC_BINARIES = fake_planner fake_localize fake_planner_fb

GRAPHICS_TARGETS = 
TARGETS = fake_planner create_base_trajectory \
	  fake_localize libfp.a simple_trajectory back_and_forth \
	  fake_planner_fb 

# rules

fake_planner:		fake_planner.o fp.o

fake_planner_fb:	fake_planner_fb.o fp.o

create_base_trajectory:	create_base_trajectory.o smoother.o spline.o

fake_localize:		fake_localize.o

libfp.a:		fp.o

simple_trajectory:	simple_trajectory.o smoother.o spline.o

back_and_forth:		back_and_forth.o fp.o

include ../../Makefile.rules
