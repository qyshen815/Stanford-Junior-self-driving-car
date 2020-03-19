/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_STREPLAN_HPP
#define AW_STREPLAN_HPP

#include "aw_StBase.hpp"
#include "aw_StActive.hpp"

namespace vlr {

/*---------------------------------------------------------------------------
 * StReplan
 *---------------------------------------------------------------------------*/
class StReplanStop;
struct StReplan : sc::state< StReplan, StActive, StReplanStop >, StBase<StReplan> {
  // on enter
	StReplan(my_context ctx);
  // on exit
  ~StReplan();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;
};

struct StReplanStop : sc::state< StReplanStop, StReplan >, StBase<StReplanStop> {
  // on enter
	StReplanStop(my_context ctx);
  // on exit
  ~StReplanStop();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;
};

/*---------------------------------------------------------------------------
 * StReroute
 *---------------------------------------------------------------------------*/
struct StReroute : sc::state< StReroute, StReplan >, StBase<StReroute> {
  // on enter
	StReroute(my_context ctx);
  // on exit
  ~StReroute();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;
};

/*---------------------------------------------------------------------------
 * StGetBackOnTrack
 *---------------------------------------------------------------------------*/
class StGetBackOnTrackPrepare;
struct StGetBackOnTrack : sc::state< StGetBackOnTrack, StReplan, StGetBackOnTrackPrepare >, StBase<StGetBackOnTrack> {
  // on enter
	StGetBackOnTrack(my_context ctx);
  // on exit
  ~StGetBackOnTrack();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;

	//! before starting the A* we wait some time so that the map becomes available
	Timestamp map_timer;
};

/*---------------------------------------------------------------------------
 * StGetBackOnTrackPrepare
 *---------------------------------------------------------------------------*/
struct StGetBackOnTrackPrepare : sc::state< StGetBackOnTrackPrepare, StGetBackOnTrack >, StBase<StGetBackOnTrackPrepare> {
  // on enter
	StGetBackOnTrackPrepare(my_context ctx);
  // on exit
  ~StGetBackOnTrackPrepare();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;

};

/*---------------------------------------------------------------------------
 * StGetBackOnTrackAStar
 *---------------------------------------------------------------------------*/
struct StGetBackOnTrackAStar : sc::state< StGetBackOnTrackAStar, StGetBackOnTrack >, StBase<StGetBackOnTrackAStar> {
  // on enter
	StGetBackOnTrackAStar(my_context ctx);
  // on exit
  ~StGetBackOnTrackAStar();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;


  Timestamp next_replan;

};

} // namespace vlr

#endif // AW_STREPLAN_HPP
