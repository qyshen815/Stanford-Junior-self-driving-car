/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team vlr
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_CHECKPOINT_H
#define AW_CHECKPOINT_H

#include "aw_netElement.h"

namespace vlr {

namespace rndf {

class WayPoint;

class CheckPoint : public NetElement {
public:
	friend class RoadNetwork;

	CheckPoint(uint32_t id, const std::string& strName);
	CheckPoint(const CheckPoint&);
	virtual ~CheckPoint(void);
	CheckPoint& operator=(const CheckPoint& other);
	CheckPoint& copy(const CheckPoint& other);

	WayPoint* wayPoint(void)	const {return m_waypoint;}
	void setWayPoint(WayPoint* wp)	{m_waypoint=wp;}
	void dump() const;

private:
	WayPoint* m_waypoint;

protected:
	friend std::ostream& operator<<(std::ostream& os, const CheckPoint& cp);
};

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const CheckPoint& cp);

typedef std::map<std::string, CheckPoint*>   TCheckPointMap;

}

} // namespace vlr

#endif


