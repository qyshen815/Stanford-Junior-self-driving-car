include ../../Makefile.conf

CXXFLAGS += 
IFLAGS += `pkg-config --cflags freetype2`

LFLAGS += -lipc_interface $(INTERFACE_LIBS) -law_geometry -law_rndf
LFLAGS += -law_rndfgl -lrndfgl -lgl_support -ltextures -limage -lglut -lIL
LFLAGS += $(GLOBAL_LIBS) -lmulticentral -lsignal_handler -lrndf

LFLAGS += -lftgl

MODULE_NAME = VEHICLESIM

SOURCES = vehiclesim.cpp vehicle.cpp multisim.cpp \
	  lasersim.cpp spawn_software.cpp spawn_centrals.cpp setup_simulation.cc \
	  rndf_lookup.cpp extract_start_state.cc stopzones.cpp place_cars.cpp \
	  stopzones.cpp trafficlights.cpp crosswalks.cpp

ROS_DEPS = opencv2 cgal

PUBLIC_INCLUDES = vehicle.h
PUBLIC_LIBRARIES = libvehicle.a
PUBLIC_BINARIES = vehiclesim multisim phantom_ibeo spawn_software \
		  spawn_centrals setup_simulation place_cars

#GRAPHICS_TARGETS = place_cars
TARGETS = vehiclesim libvehicle.a multisim spawn_software \
	  spawn_centrals setup_simulation extract_start_state

# rules

vehiclesim:	vehiclesim.o libvehicle.a

libvehicle.a:	vehicle.o

multisim:	multisim.o libvehicle.a lasersim.o stopzones.o trafficlights.o crosswalks.o

phantom_ibeo:	lasersim.o

spawn_software:	spawn_software.o

spawn_centrals:	spawn_centrals.o

setup_simulation:	setup_simulation.o

#place_cars:	place_cars.o rndf_lookup.o

extract_start_state:	extract_start_state.o

include ../../Makefile.rules
