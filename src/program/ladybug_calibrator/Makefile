include ../../Makefile.conf

MODULE_NAME = LADYBUG_CALIBRATOR

LINK = g++
LFLAGS += -lvelo_support -lladybug_video -lladybug_model -lladybug_frameparser -limage -lblf -lasync_writer -lfastjpeg -ldc1394 
LFLAGS += $(IPPLIBS) $(INTERFACE_LIBS) $(GLOBAL_LIBS)
IFLAGS += -Iepnp/

SOURCES          = hand_calibrator_pnp.cpp hand_calibrator_direct.cpp scan_projector.cpp calibration_viewer.cpp color_points.cpp format_data.cpp


ifeq (1,$(HAVE_IPP))
TARGETS          = hand_calibrator_direct hand_calibrator_pnp scan_projector calibration_viewer format_data #color_points dies on Capek
else
TARGETS = noipp
endif

PUBLIC_LIBRARIES += 
PUBLIC_BINARIES  += hand_calibrator_direct hand_calibrator_pnp calibration_viewer color_points
PUBLIC_INCLUDES  += 

ROS_DEPS = opencv2 eigen_extensions pcl pcl_ros

noipp:		;@echo " --> No IPP.  Skipping."

format_data: format_data.o

hand_calibrator_direct: hand_calibrator_direct.o

hand_calibrator_pnp: hand_calibrator_pnp.o epnp/epnp.o

scan_projector: scan_projector.o

calibration_viewer: calibration_viewer.o cropped_video_writer.o

color_points: color_points.o

include ../../Makefile.rules
