/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_STZONE_HPP
#define AW_STZONE_HPP

#include "aw_StBase.hpp"
#include "aw_StActive.hpp"

namespace vlr {

/*---------------------------------------------------------------------------
 * StZone
 *---------------------------------------------------------------------------*/
struct StZoneApproach;
struct StZone : sc::state< StZone, StActive , StZoneApproach>, StBase<StZone> {
  // on enter
  StZone(my_context ctx);
  // on exit
  ~StZone();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;

  //! list of the parking maneuers ins this zone
  std::vector<AnnotatedRouteEdge*> zone_maneuvers;

  int no_perimeter_points;
};

/*---------------------------------------------------------------------------
 * StZoneApproach
 *---------------------------------------------------------------------------*/
struct StZoneApproach: sc::state< StZoneApproach, StZone >, StBase<StZoneApproach>
{
  // on enter
  StZoneApproach(my_context ctx);
  // on exit
  ~StZoneApproach();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;
};

/*---------------------------------------------------------------------------
 * StZoneEntering
 *---------------------------------------------------------------------------*/
struct StZoneEntering : sc::state< StZoneEntering, StZone>, StBase<StZoneEntering> {
  // on enter
  StZoneEntering(my_context ctx);
  // on exit
  ~StZoneEntering();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;
};


/*---------------------------------------------------------------------------
 * StZoneParking
 *---------------------------------------------------------------------------*/
struct StZoneParking : sc::state< StZoneParking, StZone>, StBase<StZoneParking> {
  // on enter
  StZoneParking(my_context ctx);
  // on exit
  ~StZoneParking();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;
};

/*---------------------------------------------------------------------------
 * StZoneParked
 *---------------------------------------------------------------------------*/
struct StZoneParked : sc::state< StZoneParked, StZone>, StBase<StZoneParked> {
  // on enter
  StZoneParked(my_context ctx);
  // on exit
  ~StZoneParked();

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
 * StZoneDriveToExit
 *---------------------------------------------------------------------------*/
struct StZoneDriveToExit : sc::state< StZoneDriveToExit, StZone>, StBase<StZoneDriveToExit> {
  // on enter
  StZoneDriveToExit(my_context ctx);
  // on exit
  ~StZoneDriveToExit();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;
};

/*---------------------------------------------------------------------------
 * StZoneRecover
 *---------------------------------------------------------------------------*/
struct StZoneRecover : sc::state< StZoneRecover, StZone>, StBase<StZoneRecover> {
  // on enter
  StZoneRecover(my_context ctx);
  // on exit
  ~StZoneRecover();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;
};

} // namespace vlr

#endif // AW_STZONE_HPP
