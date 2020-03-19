include ../../Makefile.conf

ifeq (1, $(HAVE_PROBT))
IFLAGS += -I../../ext/probt/include
endif

IFLAGS += $(STANFORD_DRIVING_IFLAGS)

PUBLIC_INCLUDES = aw_ChsmPlanner.hpp aw_RouteSampler.hpp aw_IntersectionManager.hpp \
                  aw_StBase.hpp aw_StActive.hpp aw_StPause.hpp aw_StDrive.hpp aw_StStop.hpp aw_StIntersection.hpp \
                  aw_ChsmEvents.hpp aw_StDriveKTurn.hpp aw_StWaitForActivation.hpp aw_StError.hpp \
                  aw_StZone.hpp aw_StDrivePassObstacle.hpp aw_MergeFeasabilityCheck.hpp aw_LaneChangeManager.hpp \
                  aw_PassObstacleManager.hpp aw_StReplan.hpp aw_StLaneChange.hpp aw_StLaneChangeTypes.hpp \
                  trajectoryEvaluator.h obstaclePrediction.h collisionCheck.h \
                  aw_StIntersectionTrafficLightStop.hpp aw_StIntersectionTrafficLightWait.hpp aw_StIntersectionTrafficLightQueue.hpp \
                  intersectionPath.hpp aw_StTrafficLight.hpp aw_TrafficLightManager.hpp \
                  aw_CrosswalkManager.hpp aw_StCrosswalk.hpp

ifeq (1,$(HAVE_IPP))
  ifeq ($(ARCH), Darwin)
      CC=g++
      IFLAGS += -I/Library/Frameworks/QtCore.framework/Headers
      IFLAGS += -I/Library/Frameworks/QtGui.framework/Headers
      IFLAGS += -I/Library/Frameworks/QtOpenGL.framework/Headers
      LFLAGS += -framework QtCore -framework QtGui
  else
      IFLAGS += `pkg-config QtCore --cflags`
      LFLAGS += `pkg-config QtCore --libs`
      IFLAGS += `pkg-config QtOpenGL --cflags`
      LFLAGS += `pkg-config QtOpenGL --libs`
      IFLAGS += `pkg-config QtGui --cflags`
      LFLAGS += `pkg-config QtGui --libs`
      IFLAGS += `pkg-config QtNetwork --cflags`
      LFLAGS += `pkg-config QtNetwork --libs`
      IFLAGS += `pkg-config QtTest --cflags`
      LFLAGS += `pkg-config QtTest --libs`
  endif

  ifneq ($(ARCH), Darwin)
  LFLAGS += -Wl,--whole-archive
  endif

  ifneq ($(ARCH), Darwin)
  LFLAGS += -Wl,--no-whole-archive
  endif

  LFLAGS += $(STANFORD_DRIVING_LFLAGS)

  #CFLAGS += -DCOMPANION
  #IFLAGS += `pkg-config --cflags gtk+-2.0 gtkglextmm-1.2 opencv`
  LFLAGS += -lcurve_smoothing

  MODULE_NAME = CHSM_PLANNING
  MODULE_COMMENT = 
  SOURCES = aw_ChsmPlanner.cpp aw_RouteSampler.cpp obstacleUpdates.cpp aw_IntersectionManager.cpp \
            aw_StBase.cpp aw_StActive.cpp aw_StPause.cpp aw_StDrive.cpp aw_StStop.cpp aw_StIntersection.cpp aw_StDriveKTurn.cpp \
            aw_StWaitForActivation.cpp aw_StError.cpp aw_StZone.cpp aw_StDrivePassObstacle.cpp aw_MergeFeasabilityCheck.cpp \
            aw_LaneChangeManager.cpp aw_PassObstacleManager.cpp aw_StReplan.cpp aw_StLaneChange.cpp \
            trajectoryEvaluator.cpp obstaclePrediction.cpp collisionCheck.cpp \
            aw_StIntersectionTrafficLightStop.cpp aw_StIntersectionTrafficLightWait.cpp aw_StIntersectionTrafficLightQueue.cpp \
            intersectionPath.cpp aw_StTrafficLight.cpp aw_TrafficLightManager.cpp \
            aw_CrosswalkManager.cpp aw_StCrosswalk.cpp
                    
  PUBLIC_LIBRARIES = libaw_chsm_planner.a
  PUBLIC_BINARIES = 
  TARGETS = libaw_chsm_planner.a
else
 TARGETS = no_ipp
endif

ROS_DEPS = cgal

# rules

no_ipp: ;@echo " --> Not building aw_chsmplanner because no IPP detected"

libaw_chsm_planner.a: aw_ChsmPlanner.o aw_RouteSampler.o obstacleUpdates.o aw_IntersectionManager.o \
                      aw_StBase.o aw_StActive.o aw_StPause.o aw_StDrive.o aw_StStop.o aw_StIntersection.o aw_StDriveKTurn.o \
                      aw_StWaitForActivation.o aw_StError.o aw_StZone.o aw_StDrivePassObstacle.o aw_MergeFeasabilityCheck.o \
                      aw_LaneChangeManager.o aw_StReplan.o aw_PassObstacleManager.o aw_StLaneChange.o \
                      trajectoryEvaluator.o obstaclePrediction.o collisionCheck.o \
                      aw_StIntersectionTrafficLightStop.o aw_StIntersectionTrafficLightWait.o aw_StIntersectionTrafficLightQueue.o \
                      intersectionPath.o aw_StTrafficLight.o aw_TrafficLightManager.o \
                      aw_CrosswalkManager.o aw_StCrosswalk.o
                  
include ../../Makefile.rules
