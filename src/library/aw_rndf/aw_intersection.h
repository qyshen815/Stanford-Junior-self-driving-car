#ifndef INTERSECTION_H_
#define INTERSECTION_H_

#include "aw_lane.h"


namespace vlr {

namespace rndf {

class Intersection;

typedef std::set<Intersection*>	TIntersectionSet;


class Intersection : public rndf::NetElement
{
public:
	Intersection(uint32_t id, const std::string& strName);
	virtual ~Intersection();

	inline void addLaneSegment(LaneSegment* l) { LaneSegments.insert(l); }
	inline void remove(LaneSegment* l) { LaneSegments.erase(l); }

	inline const TLaneSegmentSet& getLaneSegments() const { return LaneSegments; }
	inline size_t numLaneSegments() const { return LaneSegments.size(); }

protected:
	TLaneSegmentSet	LaneSegments;
};

inline std::ostream& operator << (std::ostream& os, const rndf::Intersection& v)
{
	// sieht komisch ist aber so. Wer die Ursache dafür findet das bekommt ein Eis und darf es fixen.
    return os << v.name() << std::string(" ( ") << boost::lexical_cast<std::string>( v.numLaneSegments() ) << std::string(" lanes)");
//    return os << std::string(v.name()) << std::string(" ( ") << boost::lexical_cast<std::string>( v.numLaneSegments() ) << std::string(" lanes)");
}

} // namespace rndf

} // namespace vlr

#endif /*INTERSECTION_H_*/
