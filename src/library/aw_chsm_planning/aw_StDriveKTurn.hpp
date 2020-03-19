/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_STKTURN_H
#define AW_STKTURN_H

#include "aw_StBase.hpp"
#include "aw_StActive.hpp"

namespace vlr {

#define A_STAR_KTURN

#ifndef A_STAR_KTURN
class StKTurnPhase1;
class StKTurnPhase2;
class StKTurnPhase3;
#endif
struct StDrive;
struct StDriveKTurnApproach;

/*---------------------------------------------------------------------------
 * StKTurn
 *---------------------------------------------------------------------------*/
class StDriveKTurn : public sc::state< StDriveKTurn, StDrive, StDriveKTurnApproach>, public StBase<StDriveKTurn>
{
public:
	StDriveKTurn(my_context ctx);
	virtual ~StDriveKTurn();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;

	dgc_pose_t start;

	//! before starting the A* we wait some time so that the map becomes available
	Timestamp map_timer;

;

#ifdef A_STAR_KTURN
  RoutePlanner::Route::RouteEdgeList::iterator opposite_lane_edge;
#else
	double delta;
  double radius;
  double switch_distance;
  double switch_speed
#endif
};

/*---------------------------------------------------------------------------
 * StDriveKTurnApproach
 *---------------------------------------------------------------------------*/
struct StDriveKTurnApproach: public sc::state< StDriveKTurnApproach, StDriveKTurn >, public StBase<StDriveKTurnApproach>
{
  // on enter
	StDriveKTurnApproach(my_context ctx);
  // on exit
  ~StDriveKTurnApproach();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
    sc::custom_reaction< EvProcess >
  > reactions;
};

#ifdef A_STAR_KTURN
/*---------------------------------------------------------------------------
 * StDriveKTurnAStar
 *---------------------------------------------------------------------------*/
struct StDriveKTurnAStar : sc::state< StDriveKTurnAStar, StDriveKTurn>, public StBase<StDriveKTurnAStar>
{
public:
  StDriveKTurnAStar(my_context ctx);
  ~StDriveKTurnAStar();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;
};
#else
/*---------------------------------------------------------------------------
 * StDriveKTurnPhaseBaseImpl
 *---------------------------------------------------------------------------*/
struct StDriveKTurnPhaseBaseImpl {
	static void transform_curvepoints_and_define_goal(dgc_pose_t& goal, CurvePoints* curvepoints, const dgc_pose_t& start);
	static bool goal_reached(const dgc_pose_t& goal, double switch_distance, double switch_speed, const dgc_pose_t& robot_pose, double robot_speed);
};

/*---------------------------------------------------------------------------
 * StDriveKTurnPhaseBase
 *---------------------------------------------------------------------------*/
template < class Derived >
class StDriveKTurnPhaseBase : public StBase<Derived> {
protected:
	typedef StDriveKTurnPhaseBase kogmo_base;
	StDriveKTurnPhaseBase(const std::string& name) : StBase<Derived>(name) {};
	dgc_pose_t goal;
	double switch_distance;
	double switch_speed;

	void transform_curvepoints_and_define_goal(CurvePoints* curvepoints, const dgc_pose_t& start){
		StDriveKTurnPhaseBaseImpl::transform_curvepoints_and_define_goal(goal, curvepoints, start);
	};
	bool goal_reached(const dgc_pose_t& robot_pose, double robot_speed){
		return StDriveKTurnPhaseBaseImpl::goal_reached(goal, switch_distance, switch_speed, robot_pose, robot_speed);
	};
};

/*---------------------------------------------------------------------------
 * StDriveKTurnPhase1
 *---------------------------------------------------------------------------*/
struct StDriveKTurnPhase1 : sc::state< StDriveKTurnPhase1, StDriveKTurn>, StDriveKTurnPhaseBase<StDriveKTurnPhase1>
{
public:
	StDriveKTurnPhase1(my_context ctx);
	~StDriveKTurnPhase1();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;
};

/*---------------------------------------------------------------------------
 * StDriveKTurnPhase2
 *---------------------------------------------------------------------------*/
struct StDriveKTurnPhase2 : sc::state< StDriveKTurnPhase2, StDriveKTurn>, StDriveKTurnPhaseBase<StDriveKTurnPhase2>
{
public:
	StDriveKTurnPhase2(my_context ctx);
	~StDriveKTurnPhase2();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;
};

/*---------------------------------------------------------------------------
 * StDriveKTurnPhase3
 *---------------------------------------------------------------------------*/
struct StDriveKTurnPhase3 : sc::state< StDriveKTurnPhase3, StDriveKTurn>, StDriveKTurnPhaseBase<StDriveKTurnPhase3>
{
public:
	StDriveKTurnPhase3(my_context ctx);
	~StDriveKTurnPhase3();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;
};
#endif /*A_STAR_KTURN */

} // namespace vlr

#endif // AW_STKTURN_H
