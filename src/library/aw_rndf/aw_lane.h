#ifndef AW_LANE_H
#define AW_LANE_H

#include <aw_RndfId.h>
#include <aw_wayPoint.h>
#include <aw_checkPoint.h>
#include <aw_stop.h>
#include <aw_exit.h>
#include <aw_laneSegment.h>
#include <aw_SpeedLimit.h>

#define FT2M_FACTOR 0.3048

namespace vlr {

namespace rndf {

class Segment;
class Intersection;

typedef std::map<std::string,Lane*>	  TLaneMap;
typedef std::set<Lane*>				  TLaneSet;


class Lane : public NetElement
{
public:
	friend class RoadNetwork;

	static const double default_width = 4.5;

	enum eBoundaryTypes
	{
		UnknownBoundary = 0,
		NoBoundary = 1,
		SolidWhite = 2,
		BrokenWhite = 3,
		SolidYellow = 4,
		DoubleYellow = 5
	};

  enum eType{car_lane, bike_lane};

	Lane(uint32_t id, const std::string& strName, const bool isVirtual = false);
	virtual ~Lane(void);
	Lane(const Lane&);
	Lane& operator=(const Lane& other);
	Lane& copy(const Lane& other);

	void setSegment(Segment* segm) { segment_=segm; }
	Segment* getSegment(void) const { return segment_; }

  void setLaneWidth(double laneWidth) { lane_width_ = laneWidth; }
  double getLaneWidth() const { return lane_width_; }

  void setLaneType(eType t) { type_ = t; }
  eType getLaneType() const { return type_; }

	void setLeftBoundaryType(Lane::eBoundaryTypes leftBoundaryType) { left_boundary_type_ = leftBoundaryType; }
	void setRightBoundaryType(Lane::eBoundaryTypes rightBoundaryType) { right_boundary_type_ = rightBoundaryType; }

	Lane::eBoundaryTypes getLeftBoundaryType() {return left_boundary_type_; }
	Lane::eBoundaryTypes getRightBoundaryType() { return right_boundary_type_; }

	void setSpeedLimit(SpeedLimit* limit) {speed_limit_ = limit;}
	SpeedLimit* getSpeedLimit() {return speed_limit_;}
	const SpeedLimit* getSpeedLimit() const {return speed_limit_;}


	//! adds a waypoint to the Lane
	bool addWayPoint(WayPoint* pWayPoint);
	bool addWayPoint(WayPoint* pWayPoint, uint32_t insert_before);
	void removeWayPoint(WayPoint* pWayPoint);
	void removeWayPoint(uint32_t index);
	WayPoint* getWaypoint(uint32_t index) { return (index<waypoints_.size() ? waypoints_[index] : NULL); }
	WayPoint* getWaypointById(uint32_t id);
	uint32_t getWaypointIndex(const WayPoint* wp) const;
	size_t numWaypoints() { return waypoints_.size(); }


	void addExit(Exit* e);
	void removeExit(Exit* e);
	void addEntry(Exit* e);
	void removeEntry(Exit* e);

	inline void addLaneSegment(LaneSegment* ls) {
	  lane_segments_.push_back(ls);
	  bbox_valid_ = false;
	}

	LaneSegment* getLaneSegment(uint32_t index) { return (index < lane_segments_.size() ? lane_segments_[index] : NULL); }
	size_t numLaneSegments() { return lane_segments_.size(); }
	LaneSegment* getLaneSegmentWithFromPoint(WayPoint* fromPoint);
	LaneSegment* getLaneSegmentWithToPoint(WayPoint* toPoint);

	bool isVirtual() { return is_virtual_; }

	bool centerLatLon(double& clat, double& clon) const;

	static std::string boundaryToRndfString(eBoundaryTypes boundary);

	inline const TWayPointVec& wayPoints() const { return waypoints_; }
    inline const TExitMap& exits() const { return exits_; }
    inline const TExitMap& entries() const { return entries_; }
	inline const TCheckPointMap& checkpoints() const { return checkpoints_; }
	inline const TStopMap& stops() const { return stops_; }
	inline const TLaneSegmentVec& getLaneSegments() const { return lane_segments_; }

	void dump() const;

  void getBoundingBox(double& xmin, double& ymin, double& xmax, double& ymax);

  std::string nextWayPointStr() const;
//  inline std::string nextWayPointStr() const {
//    std::cout << __FUNCTION__ <<": " << segment_->name() << ", " << name() << ", "<< nextIdStr(waypoints_) << std::endl;
//    return segment_->name() + name() + nextIdStr(waypoints_);
//  }

private:
	TWayPointVec	 waypoints_;
	TExitMap			 exits_;
	TExitMap			 entries_;
	TCheckPointMap checkpoints_;
	TStopMap			 stops_;

	TLaneSegmentVec		lane_segments_;

	Segment* segment_; // parent Segment of the Lane
  SpeedLimit* speed_limit_;

	double lane_width_;
	eBoundaryTypes left_boundary_type_;
	eBoundaryTypes right_boundary_type_;
	double lat_sum_, lon_sum_, length_;

	bool is_virtual_;
  bool bbox_valid_;
  double xmin_, ymin_, xmax_, ymax_;
  eType type_;
	friend std::ostream& operator<<(std::ostream& os, const Lane& l);
};

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const Lane& l);

};

} // namespace vlr

#endif


