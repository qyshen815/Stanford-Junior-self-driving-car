#ifndef AW_ST_CROSSWALK_HPP
#define AW_ST_CROSSWALK_HPP

#include "aw_StBase.hpp"
#include "aw_StActive.hpp"
#include "aw_CrosswalkManager.hpp"

namespace vlr {

/*---------------------------------------------------------------------------
 * StCrosswalk
 *---------------------------------------------------------------------------*/
struct StCrosswalkApproach;
struct StCrosswalk : sc::state< StCrosswalk, StActive, StCrosswalkApproach>, StBase<StCrosswalk>
{
	// on enter
	StCrosswalk(my_context ctx);
	// on exit
	~StCrosswalk();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list <	sc::custom_reaction< EvProcess > > reactions;

	// internal
	CrosswalkManager* cwm_;

//private:
	// generates curvepoints taking multi-matching in intersections into account
	void generateCurvepoints(const double stop_distance, const double max_speed);
};

/*---------------------------------------------------------------------------
 * StCrosswalkApproach
 *---------------------------------------------------------------------------*/
struct StCrosswalkApproach: sc::state< StCrosswalkApproach, StCrosswalk >, StBase<StCrosswalkApproach>
{
	// on enter
	StCrosswalkApproach(my_context ctx);
	// on exit
	~StCrosswalkApproach();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;

};

/*---------------------------------------------------------------------------
 * StCrosswalkStop
 *---------------------------------------------------------------------------*/
struct StCrosswalkStop: sc::state< StCrosswalkStop, StCrosswalk >, StBase<StCrosswalkStop>
{
	// on enter
	StCrosswalkStop(my_context ctx);
	// on exit
	~StCrosswalkStop();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;
};

/*---------------------------------------------------------------------------
 * StCrosswalkWait
 *---------------------------------------------------------------------------*/
struct StCrosswalkWait: sc::state< StCrosswalkWait, StCrosswalk >, StBase<StCrosswalkWait>
{
	// on enter
	StCrosswalkWait(my_context ctx);
	// on exit
	~StCrosswalkWait();

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
 * StCrosswalkQueue
 *---------------------------------------------------------------------------*/
struct StCrosswalkQueue: sc::state< StCrosswalkQueue, StCrosswalk >, StBase<StCrosswalkQueue>
{
	// on enter
	StCrosswalkQueue(my_context ctx);
	// on exit
	~StCrosswalkQueue();

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

} // namespace vlr

#endif // AW_ST_CROSSWALK_HPP
