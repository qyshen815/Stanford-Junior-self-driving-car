include ../../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += -ltrajectory $(INTERFACE_LIBS) -lpswrap $(GLOBAL_LIBS) \
	  -lipc_interface
LINK = g++

MODULE_NAME = TRAJECTORY_UTILS

SOURCES = log_to_traj.cc record_trajectory.cc compute_velocities.cpp \
	  spline_smooth.cpp trajectory_info.cpp trajectory_to_ps.cpp \
	  trim_trajectory.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = log_to_traj record_trajectory compute_velocities \
		  spline_smooth trajectory_info trajectory_to_ps \
		  trim_trajectory

TARGETS = log_to_traj record_trajectory compute_velocities \
	  spline_smooth trajectory_info trajectory_to_ps trim_trajectory

# rules

log_to_traj:		log_to_traj.o

record_trajectory:	record_trajectory.o

compute_velocities:	compute_velocities.o

spline_smooth:		spline_smooth.o

trajectory_info:	trajectory_info.o

trajectory_to_ps:	trajectory_to_ps.o

trim_trajectory:	trim_trajectory.o

include ../../Makefile.rules
