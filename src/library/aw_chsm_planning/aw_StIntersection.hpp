/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_ST_INTERSECTION_HPP
#define AW_ST_INTERSECTION_HPP

#include "aw_StBase.hpp"
#include "aw_StActive.hpp"
#include "aw_IntersectionManager.hpp"
#include "obstaclePrediction.h"

namespace vlr {

/*---------------------------------------------------------------------------
 * StIntersection
 *---------------------------------------------------------------------------*/
struct StIntersectionApproach;
struct StIntersection : sc::state< StIntersection, StActive, StIntersectionApproach>, StBase<StIntersection>
{
	// on enter
	StIntersection(my_context ctx);
	// on exit
	~StIntersection();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list <	sc::custom_reaction< EvProcess > > reactions;

	// internal
	IntersectionManager* isec_man;

	int turnDirection;

	// hard time-limit to switch to recover mode
	Timestamp max_wait_at_intersection;

	// the last edge before the intersection that we passed - used for recover recover state to drive back to last known good position
	RndfEdge* recover_recover_edge;
	bool inIntersection;

	enum RecoverMode {
		RECOVER_TO_ENTRY,
		RECOVER_TO_EXIT
	};

	RecoverMode recover_mode;

//private:
	// generates curvepoints taking multi-matching in intersections into account
	void generateCurvepoints(const double stop_distance, const double max_speed);
};

/*---------------------------------------------------------------------------
 * StIntersectionApproach
 *---------------------------------------------------------------------------*/
struct StIntersectionApproach: sc::state< StIntersectionApproach, StIntersection >, StBase<StIntersectionApproach>
{
	// on enter
	StIntersectionApproach(my_context ctx);
	// on exit
	~StIntersectionApproach();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;

};

/*---------------------------------------------------------------------------
 * StIntersectionStop
 *---------------------------------------------------------------------------*/
struct StIntersectionStop: sc::state< StIntersectionStop, StIntersection >, StBase<StIntersectionStop>
{
	// on enter
	StIntersectionStop(my_context ctx);
	// on exit
	~StIntersectionStop();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;
};

/*---------------------------------------------------------------------------
 * StIntersectionWait
 *---------------------------------------------------------------------------*/
struct StIntersectionWait: sc::state< StIntersectionWait, StIntersection >, StBase<StIntersectionWait>
{
	// on enter
	StIntersectionWait(my_context ctx);
	// on exit
	~StIntersectionWait();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;

	// internal variables
	Timestamp stop_time;
};


/*---------------------------------------------------------------------------
 * StIntersectionPrioStop
 *---------------------------------------------------------------------------*/
struct StIntersectionPrioStop: sc::state< StIntersectionPrioStop, StIntersection >, StBase<StIntersectionPrioStop>
{
	// on enter
	StIntersectionPrioStop(my_context ctx);
	// on exit
	~StIntersectionPrioStop();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;
};

/*---------------------------------------------------------------------------
 * StIntersectionPrioWait
 *---------------------------------------------------------------------------*/
struct StIntersectionPrioWait: sc::state< StIntersectionPrioWait, StIntersection >, StBase<StIntersectionPrioWait>
{
	// on enter
	StIntersectionPrioWait(my_context ctx);
	// on exit
	~StIntersectionPrioWait();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;

};


/*---------------------------------------------------------------------------
 * StIntersectionDriveInside
 *---------------------------------------------------------------------------*/
struct StIntersectionDriveInside: sc::state< StIntersectionDriveInside, StIntersection >, StBase<StIntersectionDriveInside>
{
	// on enter
	StIntersectionDriveInside(my_context ctx);
	// on exit
	~StIntersectionDriveInside();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;

	bool intersectionEntered;
	Timestamp exitTime;
};

/*---------------------------------------------------------------------------
 * StIntersectionPrioDriveInside
 *---------------------------------------------------------------------------*/
struct StIntersectionPrioDriveInside: sc::state< StIntersectionPrioDriveInside, StIntersection >, StBase<StIntersectionPrioDriveInside>
{
	// on enter
	StIntersectionPrioDriveInside(my_context ctx);
	// on exit
	~StIntersectionPrioDriveInside();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;

	bool intersectionEntered;
	Timestamp exitTime;
};

/*---------------------------------------------------------------------------
 * StIntersectionQueue
 *---------------------------------------------------------------------------*/
struct StIntersectionQueue: sc::state< StIntersectionQueue, StIntersection >, StBase<StIntersectionQueue>
{
	// on enter
	StIntersectionQueue(my_context ctx);
	// on exit
	~StIntersectionQueue();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;

	Timestamp congestionTimeout;
	Pose congestion_last_pose_;
};


/*---------------------------------------------------------------------------
 * StIntersectionRecover
 *---------------------------------------------------------------------------*/
class StIntersectionRecoverPrepare;
struct StIntersectionRecover: sc::state< StIntersectionRecover, StIntersection, StIntersectionRecoverPrepare >, StBase<StIntersectionRecover>
{
	// on enter
	StIntersectionRecover(my_context ctx);
	// on exit
	~StIntersectionRecover();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;

	Timestamp map_timer;
};

/*---------------------------------------------------------------------------
 * StIntersectionRecoverPrepare
 *---------------------------------------------------------------------------*/
struct StIntersectionRecoverPrepare: sc::state< StIntersectionRecoverPrepare, StIntersectionRecover >, StBase<StIntersectionRecoverPrepare>
{
	// on enter
	StIntersectionRecoverPrepare(my_context ctx);
	// on exit
	~StIntersectionRecoverPrepare();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;
};

/*---------------------------------------------------------------------------
 * StIntersectionRecoverToExit
 *---------------------------------------------------------------------------*/
struct StIntersectionRecoverToExit: sc::state< StIntersectionRecoverToExit, StIntersectionRecover >, StBase<StIntersectionRecoverToExit>
{
	// on enter
	StIntersectionRecoverToExit(my_context ctx);
	// on exit
	~StIntersectionRecoverToExit();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;

	RndfEdge* targetEdge;
	dgc_pose_t target;
};

/*---------------------------------------------------------------------------
 * StIntersectionRecoverBackupToEntry
 *---------------------------------------------------------------------------*/
struct StIntersectionRecoverBackupToEntry: sc::state< StIntersectionRecoverBackupToEntry, StIntersectionRecover >, StBase<StIntersectionRecoverBackupToEntry>
{
	// on enter
	StIntersectionRecoverBackupToEntry(my_context ctx);
	// on exit
	~StIntersectionRecoverBackupToEntry();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;

	RndfEdge* targetEdge;
	dgc_pose_t target;
};

} // namespace vlr

#endif // AW_ST_INTERSECTION_HPP
