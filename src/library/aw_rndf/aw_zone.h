/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team vlr
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_ZONE_H
#define AW_ZONE_H

#include "aw_SpeedLimit.h"
#include "aw_perimeter.h"
#include "aw_spot.h"


namespace vlr {

namespace rndf
{
class Zone;

typedef std::map<std::string,Zone*>			TZoneMap;


class Zone : public NetElement
{
public:
	Zone(uint32_t id, const std::string& strName);
	Zone(const Zone&);
	virtual ~Zone(void);

	Zone& operator=(const Zone& other);
	Zone& copy(const Zone& other);

	void setDescription(std::string description) {
		m_description = description;
	}

	// add Perimeter
	bool addPerimeter(Perimeter* pPerimeter);
	void removePerimeter(Perimeter* pPerimeter);
	Perimeter* getPerimterById(uint32_t id);
	const TPerimeterMap& perimeters(void) { return m_perimeters; }
	uint32_t getNextPerimeterId() const;
	std::string getNextPerimeterStr() const;
	uint32_t numPerimeters() { return m_perimeters.size(); }

	bool addSpot(Spot* pSpot);
	void removeSpot(Spot* pSpot);
	Spot* getSpotById(uint32_t id);
	const TSpotMap& spots(void) { return m_spots; }
	uint32_t getNextSpotId() const;
	std::string getNextSpotStr() const;
	uint32_t numSpots() { return m_spots.size(); }

	void setSpeedLimit(SpeedLimit * limit) { m_speedLimit = limit; }
	SpeedLimit const * getSpeedLimit() const { return m_speedLimit; }

	bool centerLatLon(double& clat, double& clon);

	void dump();


	void setOffroad() { m_offroad = true; }
	bool getOffroad() { return m_offroad; }

protected:
	std::string m_description;
	SpeedLimit * m_speedLimit;

	TPerimeterMap m_perimeters;
	TSpotMap m_spots;
	bool m_offroad;
	friend std::ostream& operator<<(std::ostream& os, const Zone& z);
};

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const Zone& z);
}

} // namespace vlr

#endif

