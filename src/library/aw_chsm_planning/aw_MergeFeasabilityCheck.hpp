/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_MERGEFEASABILITYCHECK_HPP
#define AW_MERGEFEASABILITYCHECK_HPP

#include <vector>
#include <cassert>
#include <aw_CGAL.h>
#include <aw_RndfEdge.h>

namespace vlr {

class c2_mergePar;

class MFCIntersection {};
extern MFCIntersection mfcIntersection;
class MFCPassObstacle {};
extern MFCPassObstacle mfcPassObstacle;

/*!
 * Handles feasability checks for one single merging point. Because this class has
 * an internal state you should use one object for the same merging point over and over again.
 */
class MergeFeasabilityCheck
{
public:

	enum Result {
		Merge,
		Stop
	};

	struct Entity {
		Entity(double distance, double speed, Vehicle* veh = NULL) : distance(distance), speed(speed), veh(veh) {};
		double distance;
		double speed;
		Vehicle* veh;
	};

	static const double nominal_acceleration;

	typedef std::vector<Entity> Entities;

	//! constructs a MFC object for an intersection
	MergeFeasabilityCheck(MFCIntersection /*noname*/, Result startState, double radius);
	//! constructs a MFC object for pass obstacle
	MergeFeasabilityCheck(MFCPassObstacle /*noname*/, Result startState);
	virtual ~MergeFeasabilityCheck();

	void setState(Result s) {state = s;};
	Result getState() const {return state;};

	void setEgoGeoConstrain(double before_MP, double after_MP);
	void setOtherGeoConstrain(double before_MP, double after_MP);
	void setTimeConstrain(double ego_before_other, double other_before_ego);
	void setHysterese(double hyst);

	Result test(const Entity& ego_vehicle, const Entity& vehicle, const double velocity_desired);
	Result test(const Entity& ego_vehicle, const Entities& vehicles, const double velocity_desired);

	static Entity getEntity(const CGAL_Geometry::Point_2& merging_point, const CGAL_Geometry::Point_2& vehicle, const double yaw, const double speed);

	static Entities getEntities(RoutePlanner::RndfEdge* edge, double offset, double dist_in_from_direction, double dist_in_to_direction, bool same_lane = true);
protected:
	Result state; // need acutal state for hysterese
	c2_mergePar* par;
};

} // namespace vlr

#endif // AW_MERGEFEASABILITYCHECK_HPP
