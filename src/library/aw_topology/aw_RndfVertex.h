/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_RNDFVERTEX_H
#define AW_RNDFVERTEX_H

#include <string>
#include <iostream>
#include <aw_CGAL.h>
#include "aw_Graph.h"

using CGAL_Geometry::Point_2;

namespace vlr {

// forward declarations
struct Vehicle;

namespace RoutePlanner {

class RndfGraphBuilder;
class RndfEdge;
class RndfIntersection;

class RndfVertex : public Vertex<RndfEdge> {
public:
	friend class RndfGraph;
	friend class RndfGraphBuilder;

	RndfVertex(double lat, double lon, double x, double y, std::string name = "nameless", int id = 0);

	~RndfVertex();

	double latitude() const { return latitude_; }
	double longitude() const { return longitude_; }
	double x() const { return x_; }
	double y() const { return y_; }
	std::string name() const { return name_; }
	int checkpointId() const { return (isCheckpoint) ? checkpoint_id_ : -1; }

	bool isCheckpointVertex() const { return isCheckpoint; }
	bool isBlockedVertex() const { return isBlocked; }
  bool isStopVertex() const { return isStop; }
  bool isTrafficLightVertex() const { return traffic_lights_.size() > 0; }
  bool isCrosswalkVertex() const { return crosswalks_.size() > 0; }
	bool isExitVertex() const { return isExit; }
  bool isParkingSpotVertex() const { return isParkingSpot; }
	bool isPerimeterPointVertex() const { return isPerimeterPoint; }
	bool isVirtualVertex() const { return isVirtual; }

	int exitCount() const { return exit_count_; }
  const std::set<std::string>& trafficLightNames() const {return traffic_lights_;}
  const std::set<std::string>& crosswalkNames() const {return crosswalks_;}

	//RndfIntersection* getIntersection() const { return m_intersection; }

	void setBlocked(bool v = true) { isBlocked = v; }
  void setStop() { isStop = true; }
//  void setTrafficLight() { isTrafficLight = true; traffic_light_count_++;}
  void addTrafficLight(const std::string& tl_name) {traffic_lights_.insert(tl_name);}
  void addCrosswalk(const std::string& cw_name) {crosswalks_.insert(cw_name);}
	void setExit() { isExit = true; ++exit_count_; }
	void setCheckpoint(int id) { isCheckpoint = true; checkpoint_id_ = id; }
	void setPerimeterPoint() { isPerimeterPoint = true; }
	void setParkingSpot() { isParkingSpot = true; }
	void setVirtual() { isVirtual = true; }

	Point_2 point() const { return Point_2(x_, y_); }

	bool hasIntersectionOutEdge(const RndfIntersection* isec) const;
	bool hasIntersectionOutEdge() const;

	//	bool was_reached; // remember if this was already reached (only important for checkpoints)
protected:
	double latitude_;   //!< GPS coordinates from RNDF
	double longitude_;
	double x_;          //!< cartesian coordinates CHANGED FROM UTM TO LOCAL!!! [m]
	double y_;
	std::string name_;
	bool isStop, isExit, isCheckpoint, isPerimeterPoint, isParkingSpot, isVirtual, isBlocked;
  int exit_count_;
//  uint32_t traffic_light_count_;
	int checkpoint_id_;
  std::set<std::string> crosswalks_;
  std::set<std::string> traffic_lights_;

  //RndfIntersection* m_intersection;
	//double spotWidth; -> Edge
};

typedef std::set<RndfVertex*> TRndfVertexSet;

}

} // namespace vlr

#endif // AW_RNDFVERTEX_H
