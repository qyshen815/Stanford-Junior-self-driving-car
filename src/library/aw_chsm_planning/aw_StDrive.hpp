/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_STDRIVE_HPP
#define AW_STDRIVE_HPP

#include "aw_StBase.hpp"
#include "aw_StActive.hpp"
#include "aw_ChsmEvents.hpp"

namespace vlr {

/*---------------------------------------------------------------------------
 * StDrive
 *---------------------------------------------------------------------------*/
struct StDriveStart;
struct StIntersection;
struct StDrive: sc::state< StDrive, StActive, StDriveStart>, StBase<StDrive>
{
  // on enter
  StDrive(my_context ctx);
  // on exit
  virtual ~StDrive();

  // reactions
  sc::result react(const EvProcess& evt);
  sc::result react(const EvStop& evt);


  // reactions
  typedef mpl::list
  <
    sc::custom_reaction< EvProcess >,
    sc::custom_reaction< EvStop >,
    sc::transition< EvCloseToIntersection, StIntersection >
  > reactions;

};


/*---------------------------------------------------------------------------
 * StDriveStart
 *---------------------------------------------------------------------------*/
struct StDriveStart: sc::state< StDriveStart, StDrive >, StBase<StDriveStart>
{
  // on enter
	StDriveStart(my_context ctx);
  // on exit
  virtual ~StDriveStart();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
    sc::custom_reaction< EvProcess >
  > reactions;
};

/*---------------------------------------------------------------------------
 * StDriveOnLane
 *---------------------------------------------------------------------------*/
struct StDriveOnLane: sc::state< StDriveOnLane, StDrive >, StBase<StDriveOnLane>
{
  // on enter
  StDriveOnLane(my_context ctx);
  // on exit
  virtual ~StDriveOnLane();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
    sc::custom_reaction< EvProcess >
  > reactions;
};


/*---------------------------------------------------------------------------
 * StDriveRecover
 *---------------------------------------------------------------------------*/
class StDriveRecoverPrepare;
struct StDriveRecover: sc::state< StDriveRecover, StDrive, StDriveRecoverPrepare >, StBase<StDriveRecover>
{
  // on enter
  StDriveRecover(my_context ctx);
  // on exit
  virtual ~StDriveRecover();

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
 * StDriveRecoverPrepare
 *---------------------------------------------------------------------------*/
struct StDriveRecoverPrepare: sc::state< StDriveRecoverPrepare, StDriveRecover >, StBase<StDriveRecoverPrepare>
{
  // on enter
	StDriveRecoverPrepare(my_context ctx);
  // on exit
  virtual ~StDriveRecoverPrepare();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
    sc::custom_reaction< EvProcess >
  > reactions;
};

/*---------------------------------------------------------------------------
 * StDriveRecoverAStar
 *---------------------------------------------------------------------------*/
struct StDriveRecoverAStar: sc::state< StDriveRecoverAStar, StDriveRecover >, StBase<StDriveRecoverAStar>
{
  // on enter
	StDriveRecoverAStar(my_context ctx);
  // on exit
  virtual ~StDriveRecoverAStar();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
    sc::custom_reaction< EvProcess >
  > reactions;

	RndfEdge* targetEdge;
};

/*---------------------------------------------------------------------------
 * StDriveStop
 *---------------------------------------------------------------------------*/
struct StDriveStop: sc::state< StDriveStop, StDrive >, StBase<StDriveStop>
{
  // on enter
	StDriveStop(my_context ctx);
  // on exit
  virtual ~StDriveStop();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
    sc::custom_reaction< EvProcess >
  > reactions;
};
struct StDriveStopped: sc::state< StDriveStopped, StDrive >, StBase<StDriveStopped>
{
  // on enter
	StDriveStopped(my_context ctx);
  // on exit
  virtual ~StDriveStopped();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
    sc::custom_reaction< EvProcess >
  > reactions;

  Timestamp stop_timer;
};

} // namespace vlr

#include "aw_StDriveKTurn.hpp"
#include "aw_StDrivePassObstacle.hpp"

#endif // AW_STDRIVE_HPP
