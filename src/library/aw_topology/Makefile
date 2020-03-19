include ../../Makefile.conf

#IFLAGS += -finstrument-functions -g -O0
#LFLAGS += -lSaturn

IFLAGS += $(STANFORD_DRIVING_IFLAGS) `pkg-config ftgl freetype2 --cflags`

ifneq ($(ARCH), Darwin)
LFLAGS += -Wl,--whole-archive
endif

LFLAGS += -lcurve_smoothing

ifneq ($(ARCH), Darwin)
LFLAGS += -Wl,--no-whole-archive
endif

LFLAGS += $(STANFORD_DRIVING_LFLAGS)

MODULE_NAME = "AW Topology"

SOURCES = aw_VehicleManager.cpp aw_Graph.cpp aw_graph_tools.cpp aw_match_to_graph.cpp \
          aw_RndfGraphBuilder.cpp aw_RndfEdge.cpp aw_RndfVertex.cpp aw_RndfIntersection.cpp aw_RndfGraph.cpp aw_Route.cpp \
          aw_Topology.cpp aw_Vehicle.cpp aw_RndfGraphSearch.cpp aw_BlockadeManager.cpp

PUBLIC_INCLUDES = aw_EdgeListDistanceTransform.cpp EdgeListDistanceTransform.hpp \
                  aw_VehicleManager.hpp aw_Graph.h aw_graph_tools.hpp aw_match_to_graph.hpp aw_RndfGraphBuilder.h \
                  aw_RndfEdge.h aw_RndfVertex.h aw_RndfIntersection.h aw_RndfGraph.h aw_Route.h aw_Topology.hpp aw_Counter.h \
                  aw_LaneConnector.h aw_Vehicle.h aw_RndfGraphSearch.h aw_BlockadeManager.hpp aw_Maneuver.h
 
PUBLIC_LIBRARIES = libaw_topology.a
PUBLIC_BINARIES = 

TARGETS = libaw_topology.a

ROS_DEPS = cgal

# rules
libaw_topology.a: aw_VehicleManager.o aw_Graph.o aw_graph_tools.o aw_match_to_graph.o \
          aw_RndfGraphBuilder.o aw_RndfEdge.o aw_RndfVertex.o aw_RndfIntersection.o aw_RndfGraph.o aw_Route.o \
          aw_Topology.o aw_Vehicle.o aw_RndfGraphSearch.o aw_BlockadeManager.o


include ../../Makefile.rules

