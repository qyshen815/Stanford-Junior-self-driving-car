/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team vlr
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_PERIMETER_H
#define AW_PERIMETER_H

#include "aw_exit.h"
#include "aw_perimeterPoint.h"


namespace vlr {

namespace rndf
{
class Zone;

typedef std::map<std::string, Perimeter*>		TPerimeterMap;

class Perimeter : public NetElement
{
public:
	Perimeter(uint32_t id, const std::string& strName);
	Perimeter(const Perimeter&);
	virtual ~Perimeter(void);
	Perimeter& operator=(const Perimeter& other);
	Perimeter& copy(const Perimeter& other);

	void setZone(Zone* z) { parent_zone_ = z; }
	Zone* zone() const { return parent_zone_; }

	bool addPerimeterPoint(PerimeterPoint* pPerimeterPoint);
	bool addPerimeterPoint(PerimeterPoint* pPerimeterPoint, uint32_t insert_before);
	void removePerimeterPoint(PerimeterPoint* pPerimeterPoint);
	void removePerimeterPoint(uint32_t index);
	PerimeterPoint* perimeterPoint(uint32_t index) { return (index<perimeter_points_.size() ? perimeter_points_[index] : NULL); }
	PerimeterPoint* perimeterPointById(uint32_t id);
	uint32_t perimeterPointIndex(const PerimeterPoint* pPerimeterPoint) const;
	size_t numPerimeterPoints() { return perimeter_points_.size(); }

	bool addExit(Exit* pExit);
	void removeExit(Exit* pExit);

  inline size_t nextPerimeterPointId() const {
    return nextId(perimeter_points_);
  }

  inline std::string nextPerimeterPointStr() const {
    return name() + "." + nextIdStr(perimeter_points_);
  }

	bool centerLatLon(double& clat, double& clon) const;

	void dump();

	const TPerimeterPointVec& perimeterPoints() const { return perimeter_points_;}
	const TExitMap& exits() const { return exits_;}

protected:
 	TPerimeterPointVec  perimeter_points_;
	TExitMap            exits_;
	Zone*				        parent_zone_;

	friend std::ostream& operator<<(std::ostream& os, const Perimeter& p);
};

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const Perimeter& p);

};

} // namespace vlr

#endif


