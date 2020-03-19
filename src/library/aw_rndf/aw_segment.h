#ifndef AW_SEGMENT_H_
#define AW_SEGMENT_H_

#include <aw_RndfId.h>
#include <aw_netElement.h>
#include <aw_SpeedLimit.h>
#include <aw_lane.h>

namespace vlr {

namespace rndf {

class RoadNetwork;

typedef std::map<std::string ,Segment*>	TSegmentMap;

class Segment : public NetElement
{
public:
	friend class rndf::RoadNetwork;

	Segment(uint32_t id, const std::string& strName);
	Segment(const Segment&);
	virtual ~Segment(void);

	Segment& operator=(const Segment& other);
	Segment& copy(const Segment& other);

	void setDescription(std::string description) { description_ = description; }
	std::string& getDescription(void) { return description_; }

	// add lanes
	void addLane(Lane* pLane);
	void removeLane(Lane* pLane);
	Lane* getLaneById(uint32_t id);
	uint32_t numLanes() { return lanes_.size(); }

	bool centerLatLon(double& clat, double& clon) const;
	//double& getLength(); // TODO: make it work ...

	void setSpeedLimit(SpeedLimit * limit) { speed_limit_ = limit; }
	SpeedLimit * getSpeedLimit() { return speed_limit_; }
	SpeedLimit const * getSpeedLimit() const { return speed_limit_; }

	void dump();

	const TLaneSet& getLanes() const { return lanes_; }

	inline std::string nextLaneStr() const {
	  return name() + "." + nextIdStr(lanes_);
	}

	void setOffroad() { offroad_ = true; }
	bool getOffroad() const { return offroad_; }

private:
	std::string description_;
	TLaneSet lanes_;
	SpeedLimit* speed_limit_;
	double lat_sum_, lon_sum_, length_;
	bool offroad_;

protected:
	friend std::ostream& operator<<(std::ostream& os, const Segment& rn);
};

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const Segment& rn);
}

} // namespace vlr

#endif

