include ../../Makefile.conf

ROS_DEPS += opencv2

CFLAGS +=
IFLAGS += 
LFLAGS += -lipc_interface -lrndf -lmdf -lIL $(INTERFACE_LIBS) $(GLOBAL_LIBS)

LFLAGS += $(IPP_LIBS)

LINK = g++

MODULE_NAME = UTIL

SOURCES = popup.cpp ll_to_utm.cpp utm_to_ll.cpp convert_images.cc pyramid_images.cc \
	  create_alameda_rndf.cpp create_shoreline_rndf.cpp sim-popup.cpp \
	  dgc_autonomy_start.cc dgc_autonomy_stop.cc create_garage_rndf.cpp \
	  change_mdf.cc test_mdf.cpp runsimsoft.cpp runcent.cpp dms_to_utm.cpp \
	  regenerate_bounds.cpp reorganize_imagery.cpp

ROS_DEPS += opencv2

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = popup ll_to_utm utm_to_ll c-inc-data sim-popup \
		  change_rndf dgc_autonomy_start dgc_autonomy_stop \
		  change_mdf test_mdf runsimsoft runcent dms_to_utm \
		  regenerate_bounds reorganize_imagery

GRAPHICS_TARGETS = pyramid_images

TARGETS = popup ll_to_utm utm_to_ll convert_images c-inc-data \
	  create_alameda_rndf sim-popup change_rndf create_shoreline_rndf \
	  dgc_autonomy_start dgc_autonomy_stop create_garage_rndf \
	  change_mdf test_mdf runsimsoft runcent dms_to_utm regenerate_bounds \
	  reorganize_imagery

# rules

popup:		popup.o

sim-popup:		sim-popup.o

ll_to_utm:	ll_to_utm.o

utm_to_ll:	utm_to_ll.o

dms_to_utm:	dms_to_utm.o

convert_images:		convert_images.o

pyramid_images:		pyramid_images.o 

c-inc-data:	c-inc-data.o

create_alameda_rndf:	create_alameda_rndf.o

create_shoreline_rndf:	create_shoreline_rndf.o

create_garage_rndf:	create_garage_rndf.o

change_rndf:		change_rndf.o

dgc_autonomy_start:	dgc_autonomy_start.o

dgc_autonomy_stop:	dgc_autonomy_stop.o

change_mdf:		change_mdf.o

test_mdf:		test_mdf.o

breakup_image:		breakup_image.o

runsimsoft:		runsimsoft.o

runcent:		runcent.o

regenerate_bounds:	regenerate_bounds.o

reorganize_imagery:	reorganize_imagery.o

include ../../Makefile.rules
