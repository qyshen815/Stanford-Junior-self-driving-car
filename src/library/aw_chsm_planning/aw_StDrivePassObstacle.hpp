/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_STDRIVEPASSOBSTACLE_HPP
#define AW_STDRIVEPASSOBSTACLE_HPP


#include "aw_StBase.hpp"
#include "aw_StActive.hpp"

#include "aw_LaneChangeManager.hpp"
#include "aw_PassObstacleManager.hpp"

namespace vlr {

/*---------------------------------------------------------------------------
 * StDrivePassObstacle
 *---------------------------------------------------------------------------*/
struct StDrivePassObstacleBranch;
struct StDrivePassObstacle : sc::state< StDrivePassObstacle, StDrive, StDrivePassObstacleBranch>, StBase<StDrivePassObstacle> {
  // on enter
  StDrivePassObstacle(my_context ctx);
  // on exit
  ~StDrivePassObstacle();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;

  LaneChangeManager* lcm;
  PassObstacleManager* pom/*mes*/;
};

/*---------------------------------------------------------------------------
 * StDrivePassObstacleBranch
 *---------------------------------------------------------------------------*/
struct StDrivePassObstacleBranch : sc::state< StDrivePassObstacleBranch, StDrivePassObstacle>, StBase<StDrivePassObstacleBranch> {
  // on enter
  StDrivePassObstacleBranch(my_context ctx);
  // on exit
  ~StDrivePassObstacleBranch();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;

  Timestamp decision_time;

};


/*---------------------------------------------------------------------------
 * StDrivePassObstacleStop
 *---------------------------------------------------------------------------*/
struct StDrivePassObstacleStop : sc::state< StDrivePassObstacleStop, StDrivePassObstacle>, StBase<StDrivePassObstacleStop> {
  // on enter
  StDrivePassObstacleStop(my_context ctx);
  // on exit
  ~StDrivePassObstacleStop();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;
};


/*---------------------------------------------------------------------------
 * StDrivePassObstacleWait
 *---------------------------------------------------------------------------*/
struct StDrivePassObstacleWait : sc::state< StDrivePassObstacleWait, StDrivePassObstacle>, StBase<StDrivePassObstacleWait> {
  // on enter
  StDrivePassObstacleWait(my_context ctx);
  // on exit
  ~StDrivePassObstacleWait();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;

  Timestamp waitTimeout;
  Timestamp congestionTimeout;
};


/*---------------------------------------------------------------------------
 * StDrivePassObstaclePass
 *---------------------------------------------------------------------------*/
struct StDrivePassObstaclePass : sc::state< StDrivePassObstaclePass, StDrivePassObstacle>, StBase<StDrivePassObstaclePass> {
  // on enter
  StDrivePassObstaclePass(my_context ctx);
  // on exit
  ~StDrivePassObstaclePass();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;
};


/*---------------------------------------------------------------------------
 * StDrivePassObstacleChangeLanes
 *---------------------------------------------------------------------------*/
struct StDrivePassObstacleChangeLanes : sc::state< StDrivePassObstacleChangeLanes, StDrivePassObstacle>, StBase<StDrivePassObstacleChangeLanes> {
  // on enter
  StDrivePassObstacleChangeLanes(my_context ctx);
  // on exit
  ~StDrivePassObstacleChangeLanes();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;
};

} // namespace vlr

#endif // STDRIVEPASSOBSTACLE_HPP
