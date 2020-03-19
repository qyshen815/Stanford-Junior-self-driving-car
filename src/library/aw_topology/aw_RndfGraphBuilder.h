/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_RNDF_GRAPH_BUILDER_H
#define AW_RNDF_GRAPH_BUILDER_H

//#include "CoordinateTransformation.h"
#include <aw_roadNetwork.h>
#include "aw_RndfGraph.h"

namespace vlr {

namespace rndf {
  class RoadNetwork;
  class Mission;
//  class WayPoint;
//  class GeoPointContainer;
  class Lane;
  class Zone;
  class exit;
}

namespace RoutePlanner {

  class RndfGraphBuilder {
  public:
    RndfGraphBuilder( rndf::RoadNetwork * rn );//, COORDINATE_TRAFO * transf);
    virtual ~RndfGraphBuilder() { }

    RndfVertex* addVertex(RndfGraph * graph, double lat, double lon, double x, double y, const std::string& name);
    RndfVertex* addVertex(RndfGraph * graph, rndf::WayPoint * point);
    RndfVertex* addVertex(RndfGraph * graph, rndf::PerimeterPoint* point);
    double computeWeight(const RndfVertex * p1, const RndfVertex * p2, double speed);
    RndfEdge* addEdge(RndfGraph * graph, const rndf::WayPoint * p1, const rndf::WayPoint * p2, std::string name, bool isLane, double minSpeed, double maxSpeed, bool isOffroad, int id);
    RndfEdge* addEdge(RndfGraph * graph, const rndf::PerimeterPoint * p1, const rndf::WayPoint * p2, std::string name, bool isLane, double minSpeed, double maxSpeed, bool isOffroad, int id);
    RndfEdge* addEdge(RndfGraph * graph, const rndf::WayPoint * p1, const rndf::PerimeterPoint * p2, std::string name, bool isLane, double minSpeed, double maxSpeed, bool isOffroad, int id);
    RndfEdge* addEdge(RndfGraph * graph, const rndf::PerimeterPoint * p1, const rndf::PerimeterPoint * p2, std::string name, bool isLane, double minSpeed, double maxSpeed, bool isOffroad, int id);
    RndfEdge* addLaneChangeEdge(RndfGraph * graph, const rndf::WayPoint * p1, const rndf::WayPoint * p2, double minSpeed, double maxSpeed, bool isOffroad);

    void addPoints(RndfGraph * graph, const rndf::TWayPointMap * waypoints, bool Perimeter, bool Spot);
    void addPoints(RndfGraph * graph, const rndf::TWayPointVec* waypoints, bool Perimeter, bool Spot);
    void addPoints(RndfGraph * graph, const rndf::TPerimeterPointVec * perimeterpoints);
    void addCheckpoints(RndfGraph& graph, const rndf::TCheckPointMap& checkpoints);
    void addStops(RndfGraph& graph, const rndf::TStopMap& stops);
    void addTrafficLights(RndfGraph& graph, const rndf::TTrafficLightMap& tlmap);
    void addCrosswalks(RndfGraph& graph, const rndf::TCrosswalkMap& cwmap);
    void addExits(RndfGraph * graph, const rndf::TExitMap * exits, double minSpeed, double maxSpeed);
    void connectAll(RndfGraph * graph, const rndf::TWayPointMap*, const rndf::WayPoint * p2, std::string prefix, double minSpeed, double maxSpeed, bool isOffroad, bool zone_);
    void connectAll(RndfGraph * graph, const rndf::TWayPointMap*, std::string prefix, double minSpeed, double maxSpeed, bool isOffroad, bool Zone, double width);
    void connectAll(RndfGraph * graph, const rndf::TWayPointVec*, const rndf::WayPoint * p2, std::string prefix, double minSpeed, double maxSpeed, bool isOffroad, bool zone_);
    void connectAll(RndfGraph * graph, const rndf::TWayPointVec*, std::string prefix, double minSpeed, double maxSpeed, bool isOffroad, bool Zone, double width);
    void connectAll(RndfGraph * graph, const rndf::TPerimeterPointMap* perimeterpoints, std::string prefix, double minSpeed, double maxSpeed, bool isOffroad);
    void connectAll(RndfGraph * graph, const rndf::TPerimeterPointMap* perimeterpoints, const rndf::WayPoint * p2, std::string prefix, double minSpeed, double maxSpeed, bool isOffroad);
    RndfGraph* buildGraph();

    double getMinSpeed(rndf::Lane * lane_);
    double getMaxSpeed(rndf::Lane * lane_);
    double getMinSpeed(rndf::Zone * zone_);
    double getMaxSpeed(rndf::Zone * zone_);

    void transformCoordsGPStoUTM(double lat, double lon, double & x, double & y);

    void addLaneChangeEdges(RndfGraph * graph);


private:
    void addVirtualLanes(RndfGraph * graph);
    void addVirtualLanes(RndfGraph * graph, rndf::Lane* mylane, double minSpeed, double maxSpeed);
    void addVirtualLanes(RndfGraph * graph, rndf::Perimeter* myperimeter, double minSpeed, double maxSpeed);

    void addAdditionalKTurnEdges();

    void addAdjacentLanes(RndfGraph& graph, const rndf::TLaneMap& lanes);

  private:
    rndf::RoadNetwork* network;
    //    rndf::Mission * mission;
    static const double max_default_edge_speed_; // = dgc_mph2ms(25);  // maximum speed for edges with unspecified speed limit
  };

}

} // namespace vlr

#endif
