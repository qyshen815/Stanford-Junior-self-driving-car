include ../../Makefile.conf

ROS_DEPS += opencv2
 
CFLAGS +=
IFLAGS += `pkg-config --cflags freetype2`
LFLAGS += -lipc_interface -lrndf -lgnuplot $(INTERFACE_LIBS) \
	  $(GRAPHICS_LIBS) $(GLOBAL_LIBS)

LFLAGS += -lIL -lftgl

LFLAGS += $(IPP_LIBS)

MODULE_NAME = LOG_UTILS

SOURCES = 		log_info.cc log_bounds.cc log_cut.cc log_strip.cc \
	  		plot_cpu.cpp plot_process.cpp plot_velocity.cpp \
			race_stats.cpp log_index.cpp \
			log_to_gmaps_overlay.cpp

ROS_DEPS += opencv2

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 	log_info log_bounds log_cut log_strip \
		  	plot_cpu plot_process plot_velocity log_index \
		  	race_stats log_to_gmaps_overlay

GRAPHICS_TARGETS = 	log_to_gmaps_overlay
TARGETS = 		log_info log_bounds log_cut log_strip plot_cpu \
	  		plot_process plot_velocity race_stats log_index 

# rules

log_info:	log_info.o

log_bounds:	log_bounds.o

log_cut:	log_cut.o

log_strip:	log_strip.o

log_index:	log_index.o

plot_cpu:	plot_cpu.o

plot_process:	plot_process.o

plot_velocity:	plot_velocity.o

plot_lag:	plot_lag.o

race_stats:	race_stats.o

log_to_gmaps_overlay:	log_to_gmaps_overlay.o

#webdir:		webdir.o webgoogle.o	// google interface outdated; still uses ImageMagick

include ../../Makefile.rules
