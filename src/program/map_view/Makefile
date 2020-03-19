include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += -lestop_interface -lapplanix_interface -llocalize_interface \
	  -libeo_interface -lparam_interface -lcan_interface \
	  -lradar_interface -lclientmap -lperception_interface -lgls_interface \
	  -lcontroller_interface -lerror_interface -lplanner_interface \
	  -lsimulator_interface -lpassat_interface -lldlrs_interface \
	  -lgrid -llogio -lgloverlay -limagery -lbil -ltextures -ltrajectory \
	  -lkdtree -lpswrap -lrndf -limage -llltransform -lproj \
	  -lrndfgl -lgui3D -lproject -lpointlist -lfacelist -lgl_support \
	  -lglui -lglut -lGL -lGLU -L/usr/X11R6/lib -lX11 -lXext -lXi \
	  -lXmu -lz -lMagick -lglobal -lipc -lz

ifeq (1,$(HAVE_IPP))
LFLAGS += -L$(IPPROOT)/lib -L$(IPPROOT)/sharedlib -ljpeg.ipp -lippj -lippi \
          -lipps -lippcore 
endif

ifeq (1,$(HAVE_VIDEOOUT))
LFLAGS += -lvideoout -lavcodec -lavformat 
endif

LINK = g++

MODULE_NAME = MAP_VIEW

SOURCES = 		map_view.cpp map_notes.cpp project_sensors.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 	map_view map_notes

TARGETS = 		map_view map_notes

# rules

map_view:		map_view.o project_sensors.o

map_notes:		map_notes.o 

include ../../Makefile.rules
