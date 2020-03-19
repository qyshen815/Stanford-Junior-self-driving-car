include ../../Makefile.conf

#CXXFLAGS += -O0 -ggdb3
IFLAGS += `pkg-config --cflags freetype2`

# Dynamic Libs
LFLAGS += $(GLOBAL_LIBS) $(INTERFACE_LIBS) $(IPPLIBS) \
          -lncurses -lpthread -llapack -lgsl -lgslcblas \
          -lrt -lm -lIL -lftgl

# Global Static Libs

# Ext Static Libs
LFLAGS += -lipc  

# Race-project src/library Static Libs
LFLAGS += -law_rndf -law_geometry -lb64 -lblf \
          -llogio -ldgc_curses -lglobal -lsignal_handler -llltransform \
          -lrndf -lshm_wrapper -lcamplayer -lllfplayer -lvlfplayer \
          -lgui2D -law_rndfgl -lgl_support

# Race-project src/interface Static Libs
LFLAGS += -lapplanix_interface -lasync_writer -lipc_interface \
          -lmulticentral -ltimesync_interface -lcamera_interface 

MODULE_NAME = TRAFFIC_LIGHTS

SOURCES          +=  traffic_light_view.cpp traffic_light_notify.cpp \
                     send_light_message.cpp traffic_light_label.cpp \
		traffic_light_transforms.cpp detection_grid.cpp \
		image_data.cpp histogram_filter_tl.cpp #\ 
#		test_image_data.cpp \

PUBLIC_BINARIES  +=  traffic_light_notify traffic_light_view send_light_message traffic_light_label #test_image_data
GRAPHICS_TARGETS +=  traffic_light_notify traffic_light_view send_light_message traffic_light_label #test_image_data

traffic_light_notify: traffic_light_notify.o traffic_light_transforms.o image_data.o detection_grid.o histogram_filter_tl.o 
traffic_light_view: traffic_light_view.o traffic_light_transforms.o
send_light_message: send_light_message.o
traffic_light_label: traffic_light_label.o
#test_image_data: test_image_data.o traffic_light_transforms.o image_data.o detection_grid.o

ROS_DEPS = opencv2 cgal

include ../../Makefile.rules
