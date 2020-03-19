#ifndef AW_STTRAFFICLIGHT_HPP_
#define AW_STTRAFFICLIGHT_HPP_

#include "aw_StBase.hpp"
#include "aw_StActive.hpp"
#include "aw_TrafficLightManager.hpp"

namespace vlr {

/*---------------------------------------------------------------------------
 * StTrafficLight
 *---------------------------------------------------------------------------*/
struct StTrafficLightApproach;
struct StTrafficLight : sc::state< StTrafficLight, StActive, StTrafficLightApproach>, StBase<StTrafficLight>
{
  // on enter
  StTrafficLight(my_context ctx);
  // on exit
  ~StTrafficLight();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list < sc::custom_reaction< EvProcess > > reactions;

  // internal
  TrafficLightManager* tlm_;

//private:
  // generates curvepoints taking multi-matching in intersections into account
  void generateCurvepoints(const double stop_distance, const double max_speed);
};

/*---------------------------------------------------------------------------
 * StTrafficLightApproach
 *---------------------------------------------------------------------------*/
struct StTrafficLightApproach: sc::state< StTrafficLightApproach, StTrafficLight >, StBase<StTrafficLightApproach>
{
  // on enter
  StTrafficLightApproach(my_context ctx);
  // on exit
  ~StTrafficLightApproach();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;

};

/*---------------------------------------------------------------------------
 * StTrafficLightStop
 *---------------------------------------------------------------------------*/
struct StTrafficLightStop:
        sc::state< StTrafficLightStop, StTrafficLight >,
        StBase<StTrafficLightStop>
{
  // on enter
  StTrafficLightStop(my_context ctx);
  // on exit
  ~StTrafficLightStop();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;

  // internal variables
};

/*---------------------------------------------------------------------------
 * StTrafficLightWait
 *---------------------------------------------------------------------------*/
struct StTrafficLightWait:
        sc::state< StTrafficLightWait, StTrafficLight>,
        StBase<StTrafficLightWait>
{
  // on enter
  StTrafficLightWait(my_context ctx);
  // on exit
  ~StTrafficLightWait();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;

  // internal variables
};

/*---------------------------------------------------------------------------
 * StTrafficLightQueue
 *---------------------------------------------------------------------------*/
struct StTrafficLightQueue: sc::state< StTrafficLightQueue, StTrafficLight >, StBase<StTrafficLightQueue>
{
  // on enter
  StTrafficLightQueue(my_context ctx);
  // on exit
  ~StTrafficLightQueue();

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

#endif // AW_STTRAFFICLIGHT_HPP_
