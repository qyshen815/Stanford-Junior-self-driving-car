#ifndef AW_STINTERSECTIONTRAFFICLIGHTQUEUE_HPP_
#define AW_STINTERSECTIONTRAFFICLIGHTQUEUE_HPP_

#include "aw_StBase.hpp"
#include "aw_StActive.hpp"
#include "aw_StIntersection.hpp"

namespace vlr {

/*---------------------------------------------------------------------------
 * StIntersectionTrafficLightQueue
 *---------------------------------------------------------------------------*/
struct StIntersectionTrafficLightQueue:
        sc::state< StIntersectionTrafficLightQueue, StIntersection >,
        StBase<StIntersectionTrafficLightQueue>
{
  // on enter
  StIntersectionTrafficLightQueue(my_context ctx);
  // on exit
  ~StIntersectionTrafficLightQueue();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;

  // internal variables
  Timestamp congestionTimeout;
  Pose congestion_last_pose_;
};

} // namespace vlr

#endif // AW_STINTERSECTIONTRAFFICLIGHTQUEUE_HPP_
