include ../../Makefile.conf

IFLAGS += $(STANFORD_DRIVING_IFLAGS)
LFLAGS += $(STANFORD_DRIVING_LFLAGS)

MODULE_NAME = "Poly Trajectory Generation"

SOURCES = keepVelocityTrajSet.cpp keep_lane_traj_set.cpp \
	  keepLaneAndVelocitySet2D.cpp keepLaneAndTrackPointSet2D.cpp \
	  trackPointTrajSet.cpp polyTraj.cpp polyTraj2D.cpp rootTraj.cpp horner.cpp
		 

PUBLIC_INCLUDES = keepVelocityTrajSet.h keep_lane_traj_set.h \
		  		  keepLaneAndVelocitySet2D.h keepLaneAndTrackPointSet2D.h \
		  		  trackPointTrajSet.h polyTraj.h polyTraj2D.h rootTraj.h horner.h \
		  		  poly_traj.h poly_traj_inlines.h
 
PUBLIC_LIBRARIES = libpoly_traj.a
PUBLIC_BINARIES = 

TARGETS = libpoly_traj.a

# rules
libpoly_traj.a: keepVelocityTrajSet.o keep_lane_traj_set.o keepLaneAndVelocitySet2D.o \
		keepLaneAndTrackPointSet2D.o trackPointTrajSet.o polyTraj.o polyTraj2D.o \
		rootTraj.o horner.o


include ../../Makefile.rules
