include ../../Makefile.conf

CFLAGS += 

IFLAGS += $(EIGEN2_IFLAGS)

LFLAGS += -lapplanix_interface -lcan_interface -lpassat_interface \
	  -lparam_interface -lvehicle $(INTERFACE_LIBS) $(GLOBAL_LIBS) \
	  -ltimesync_interface -llogio -ljoystick -lglobal -lipc \
	  -lipc_interface -lpoly_traj \

LINK = g++

MODULE_NAME = CONTROLLER_LQR

SOURCES = controller_mpc.cpp controller_log_viewer.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = controller controller_log_viewer

GRAPHICS_TARGETS =

TARGETS = controller controller_log_viewer

ROS_DEPS = eigen

no_eigen: ;@echo " --> Eigen2 library not found, not building controller"

controller: controller_mpc.o
controller_log_viewer: controller_log_viewer.o

include ../../Makefile.rules
