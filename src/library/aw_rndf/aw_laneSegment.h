#ifndef AW_LANESEGMENT_H
#define AW_LANESEGMENT_H

namespace vlr {

namespace rndf {

class Segment;
class Intersection;
class WayPoint;
class Lane;
class LaneSegment;

typedef std::vector<LaneSegment*>           TLaneSegmentVec;
typedef std::set<LaneSegment*>              TLaneSegmentSet;

class  LaneSegment {
public:
    LaneSegment(Lane* _lane, WayPoint* _fromPoint = NULL, WayPoint* _toPoint = NULL) :
		lane_(_lane), from_way_point_(_fromPoint), to_way_point_(_toPoint), intersection_(NULL), stop_lane_(false), kturn_lane_(false)
	{}

	std::string name() const;
    inline WayPoint* fromWayPoint() const {return from_way_point_;}
    inline WayPoint* toWayPoint() const {return to_way_point_;}

    inline const Lane* lane() const {return lane_;}
    inline Lane* lane() {return lane_;}
    inline Intersection*& intersection() {return intersection_;}

    inline TLaneSegmentSet& nextLaneSegments() {return next_lane_segments_;}
    inline TLaneSegmentSet& prevLaneSegments() {return prev_lane_segments_;}
    inline TLaneSegmentSet& leftLaneSegments() {return left_lane_segments_;}
    inline TLaneSegmentSet& rightLaneSegments() {return right_lane_segments_;}
    inline TLaneSegmentSet& oncomingLaneSegments() {return oncoming_lane_segments_;}
    inline TLaneSegmentSet& crossingLaneSegments() {return crossing_lane_segments_;}

	bool hasNextLaneSegment(LaneSegment* ls) { return (next_lane_segments_.find(ls) != next_lane_segments_.end()); }
	bool hasPrevLaneSegment(LaneSegment* ls) { return (prev_lane_segments_.find(ls) != prev_lane_segments_.end()); }

	bool hasLeftLaneSegment(LaneSegment* ls) { return (left_lane_segments_.find(ls) != left_lane_segments_.end()); }
	bool hasRightLaneSegment(LaneSegment* ls) { return (right_lane_segments_.find(ls) != right_lane_segments_.end()); }
	bool hasOncomingLaneSegment(LaneSegment* ls) { return (oncoming_lane_segments_.find(ls) != oncoming_lane_segments_.end()); }

	inline bool& stopLane() {return stop_lane_;}
	inline bool& kturnLane() {return kturn_lane_;}
	inline bool isStopLane() const { return stop_lane_; }
	inline bool isPriorityLane() const { return !stop_lane_; }
	inline bool isKTurnEdge() const { return kturn_lane_; }

private:
	Lane* lane_;                               // parent Lane
	WayPoint* from_way_point_;                 // waypoint where the Lane segement starts
	WayPoint* to_way_point_;                   // waypoint where the Lane segement ends
	Intersection* intersection_;               // parent intersection
	TLaneSegmentSet next_lane_segments_;       // list of following lane segments
	TLaneSegmentSet prev_lane_segments_;       // list of previous lane segments
	TLaneSegmentSet left_lane_segments_;       // left adjacent lane segments in same direction
	TLaneSegmentSet right_lane_segments_;      // right adjacent lane segments
	TLaneSegmentSet oncoming_lane_segments_;   // left adjacent lane segments in oncoming direction
	TLaneSegmentSet crossing_lane_segments_;   // lane segments that cross this lane segment

	bool stop_lane_;
	bool kturn_lane_;
};


// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const Lane& l);

};

} // namespace vlr

#endif  // AW_LANESEGMENT_H


