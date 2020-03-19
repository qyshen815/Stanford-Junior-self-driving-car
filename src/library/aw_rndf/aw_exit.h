/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team vlr
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef _EXIT_H_
#define _EXIT_H_

#include "aw_netElement.h"

namespace vlr {

namespace rndf
{
class Lane;
class Perimeter;
class WayPoint;
class PerimeterPoint;
class Exit;

typedef std::map<std::string, Exit*>	TExitMap;


class Exit : public NetElement
{
public:
	friend class RoadNetwork;

	enum eExitTypes {LaneToLane,LaneToPerimeter,PerimeterToLane, PerimeterToPerimeter};

	Exit(uint32_t id, const std::string& strName);
	Exit(const Exit&);
	virtual ~Exit(void);
	Exit& operator=(const Exit& other);
	Exit& copy(const Exit& other);

	void setExitType(Exit::eExitTypes exitType) { m_exitType = exitType; }
	eExitTypes exitType() { return m_exitType; }

	WayPoint* getExitFromLane() {return m_exitFromWayPoint;}
	PerimeterPoint* getExitFromPerimeter() {return m_exitFromPerimeterPoint;}
	WayPoint* getExitToLane() {return m_exitToWayPoint;}
	PerimeterPoint* getExitToPerimeter() {return m_exitToPerimeterPoint;}

	void setExitFrom(WayPoint* exitFrom) {m_exitFromWayPoint = exitFrom;}
	void setExitFrom(PerimeterPoint* exitFrom) {m_exitFromPerimeterPoint = exitFrom;}
	void setExitTo(WayPoint* exitTo) {m_exitToWayPoint = exitTo;}
	void setExitTo(PerimeterPoint* exitTo) {m_exitToPerimeterPoint = exitTo;}

	void dump() const;

private:
	WayPoint*       m_exitFromWayPoint;
	WayPoint*       m_exitToWayPoint;
	PerimeterPoint* m_exitFromPerimeterPoint;
	PerimeterPoint* m_exitToPerimeterPoint;
	eExitTypes      m_exitType;

protected:
	friend std::ostream& operator<<(std::ostream& os, const Exit& e);
};

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const Exit& e);

};

} // namespace vlr

#endif


