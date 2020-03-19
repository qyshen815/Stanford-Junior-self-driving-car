/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_STLANECHANGE_HPP
#define AW_STLANECHANGE_HPP

#include "aw_StBase.hpp"
#include "aw_StActive.hpp"
#include "aw_StLaneChangeTypes.hpp"
#include <aw_graph_tools.hpp>
//#include <aw_masterofthetentacles.h>

namespace vlr {

struct StPrepareLaneChange;

using GraphTools::PlaceOnGraph;


/*---------------------------------------------------------------------------
 * StLaneChange
 *---------------------------------------------------------------------------*/
struct StLaneChange : sc::state< StLaneChange, StDrive, StPrepareLaneChange >, StBase<StLaneChange>
{
	// on enter
	StLaneChange(my_context ctx);
	// on exit
	~StLaneChange();

	//StLaneChange(const StLaneChange& src); - dont know how to do that

	// reactions
	sc::result react(const EvProcess& evt);
	sc::result react(const EvPause& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >,
	sc::custom_reaction< EvPause >
	> reactions;


	bool mergePossible(double desired_speed, bool check_only_forward = true);

	bool placeIsValid(const GraphPlace& place) const;
	bool snailThroughPossible( const Vehicle& base_vehicle, StLaneChangeLaneChangeType change_type ) const;

//	void setLaneChangeTentacleParams();
//	void storeTentacleParams();
//	void restoreTentacleParams();

	void restorePauseState();


	PlaceOnGraph change_point;
	GraphPlace merge_point;
	GraphPlace merge_end_point;
	StLaneChangeLaneChangeReason change_reason;
	StLaneChangeLaneChangeType change_type;
	double lateral_offset;               // offset between the two lanes
	double change_length;                // length of the change
	double merge_length;                 // length of the merging zone
	bool merge_allowed;
	bool has_to_stop;
	StLaneChangeRecoverType recover_type;

//	MasterOfTheTentacles::Parameters old_params;


	MergeFeasabilityCheck* mfc;
	std::map<Vehicle*, Timestamp> obstacles_in_merge_zone;
	std::map<Vehicle*, Timestamp> merging_suckers;	// cars that prohibit lane changing due to negativ mergecheck
};

/*---------------------------------------------------------------------------
 * StPrepareLaneChange
 *---------------------------------------------------------------------------*/
struct StPrepareLaneChange : sc::state< StPrepareLaneChange, StLaneChange >, StBase<StPrepareLaneChange>
{
	// on enter
	StPrepareLaneChange(my_context ctx);
	// on exit
	~StPrepareLaneChange();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;

	Timestamp prep_start;
};

/*---------------------------------------------------------------------------
 * StChangeToLeftLane
 *---------------------------------------------------------------------------*/
struct StChangeToLeftLane : sc::state< StChangeToLeftLane, StLaneChange >, StBase<StChangeToLeftLane>
{
	// on enter
	StChangeToLeftLane(my_context ctx);
	// on exit
	~StChangeToLeftLane();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;
};

/*---------------------------------------------------------------------------
 * StChangeToRightLane
 *---------------------------------------------------------------------------*/
struct StChangeToRightLane : sc::state< StChangeToRightLane, StLaneChange >, StBase<StChangeToRightLane>
{
	// on enter
	StChangeToRightLane(my_context ctx);
	// on exit
	~StChangeToRightLane();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;
};

/*---------------------------------------------------------------------------
 * StChangeToLeftOppositeLane
 *---------------------------------------------------------------------------*/
struct StChangeToLeftOppositeLane : sc::state< StChangeToLeftOppositeLane, StLaneChange >, StBase<StChangeToLeftOppositeLane>
{
	// on enter
	StChangeToLeftOppositeLane(my_context ctx);
	// on exit
	~StChangeToLeftOppositeLane();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;
};

/*---------------------------------------------------------------------------
 * StDriveOnOppositeLane
 *---------------------------------------------------------------------------*/
struct StDriveOnOppositeLane : sc::state< StDriveOnOppositeLane, StLaneChange >, StBase<StDriveOnOppositeLane>
{
	// on enter
	StDriveOnOppositeLane(my_context ctx);
	// on exit
	~StDriveOnOppositeLane();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;
};


/*---------------------------------------------------------------------------
 * StChangeBackFromOppositeLane
 *---------------------------------------------------------------------------*/
struct StChangeBackFromOppositeLane : sc::state< StChangeBackFromOppositeLane, StLaneChange >, StBase<StChangeBackFromOppositeLane>
{
	// on enter
	StChangeBackFromOppositeLane(my_context ctx);
	// on exit
	~StChangeBackFromOppositeLane();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;
};


/*---------------------------------------------------------------------------
 * StChangeBackFromOppositeLane
 *---------------------------------------------------------------------------*/
struct StAbortLaneChange : sc::state< StAbortLaneChange, StLaneChange >, StBase<StAbortLaneChange>
{
	// on enter
	StAbortLaneChange(my_context ctx);
	// on exit
	~StAbortLaneChange();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;
};


/*---------------------------------------------------------------------------
 * StLaneChangeRecover
 *---------------------------------------------------------------------------*/
class StLaneChangeRecoverPrepare;
struct StLaneChangeRecover: sc::state< StLaneChangeRecover, StLaneChange, StLaneChangeRecoverPrepare >, StBase<StLaneChangeRecover>
{
	// on enter
	StLaneChangeRecover(my_context ctx);
	// on exit
	~StLaneChangeRecover();

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
 * StLaneChangeRecoverPrepare
 *---------------------------------------------------------------------------*/
struct StLaneChangeRecoverPrepare: sc::state< StLaneChangeRecoverPrepare, StLaneChangeRecover >, StBase<StLaneChangeRecoverPrepare>
{
	// on enter
	StLaneChangeRecoverPrepare(my_context ctx);
	// on exit
	~StLaneChangeRecoverPrepare();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;
};

/*---------------------------------------------------------------------------
 * StLaneChangeRecoverAStar
 *---------------------------------------------------------------------------*/
struct StLaneChangeRecoverAStar: sc::state< StLaneChangeRecoverAStar, StLaneChangeRecover >, StBase<StLaneChangeRecoverAStar>
{
	// on enter
	StLaneChangeRecoverAStar(my_context ctx);
	// on exit
	~StLaneChangeRecoverAStar();

	// reactions
	sc::result react(const EvProcess& evt);

	// reactions
	typedef mpl::list
	<
	sc::custom_reaction< EvProcess >
	> reactions;

	dgc_pose_t target;
	bool failed;
};


//=============================================================================
//		Convenience Functions
//=============================================================================

bool leftLaneAllowsLaneChange(const PlaceOnGraph& lc_start, double length);
bool rightLaneAllowsLaneChange(const PlaceOnGraph& lc_start, double length);
bool leftOppositeLaneAllowsLaneChange(const PlaceOnGraph& lc_start, double length);

} // namespace vlr

#endif // AW_STLANECHANGE_HPP
