/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team vlr
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef _PERIMETERPOINT_H_
#define _PERIMETERPOINT_H_

#include "aw_netElement.h"
#include "aw_exit.h"


namespace vlr {

namespace rndf
{
class Perimeter;
class Exit;

typedef std::map<RndfId, PerimeterPoint*, RndfIdLess>	TPerimeterPointMap;
typedef std::vector<PerimeterPoint*> TPerimeterPointVec;

class PerimeterPoint : public NetElement
{
public:
	typedef std::map<std::string, rndf::Exit*> TExitMap;

	PerimeterPoint(uint32_t id, const std::string& strName);
	PerimeterPoint(const PerimeterPoint&);
	virtual ~PerimeterPoint(void);
	PerimeterPoint& operator=(const PerimeterPoint& other);
	PerimeterPoint& copy(const PerimeterPoint& other);

	void setPerimeter(Perimeter* p) { m_parentPerimeter = p; }
	const Perimeter* getPerimeter() const { return m_parentPerimeter; }

	void addExit(Exit* e);
	void removeExit(Exit* e);
	void addEntry(Exit* e);
	void removeEntry(Exit* e);

	const TExitMap& exits(void) const { return m_exits; }
	const TExitMap& entries(void) const { return m_entries; }

	void setLatLon(double lat, double lon);
	void setUtm(double utm_x, double utm_y,  char* utm_zone);

	uint32_t index() const;

	double lat() const { return m_coordinate_lat; };
	double lon() const { return m_coordinate_lon; };
	double utm_x() const { return m_coordinate_utm_x; };
	double utm_y() const { return m_coordinate_utm_y; };
	char* utm_zone(void) { return m_coordinate_utm_zone; };
	double x(void) const { return m_coordinate_utm_x; };
	double y(void) const { return m_coordinate_utm_y; };

	virtual void dump();

protected:
	Perimeter* m_parentPerimeter;

	TExitMap m_exits;
	TExitMap m_entries;

	double m_coordinate_lat;
	double m_coordinate_lon;
	double m_coordinate_utm_x;
	double m_coordinate_utm_y;
	char   m_coordinate_utm_zone[10];

	friend std::ostream& operator<<(std::ostream& os, const PerimeterPoint& p);
};

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const PerimeterPoint& p);

};

} // namespace vlr

#endif
