#ifndef AW_STINTERSECTIONTRAFFICLIGHTSTOP_HPP_
#define AW_STINTERSECTIONTRAFFICLIGHTSTOP_HPP_

#include "aw_StBase.hpp"
#include "aw_StActive.hpp"
#include "aw_StIntersection.hpp"

namespace vlr {

/*---------------------------------------------------------------------------
 * StIntersectionTrafficLightStop
 *---------------------------------------------------------------------------*/
struct StIntersectionTrafficLightStop:
        sc::state< StIntersectionTrafficLightStop, StIntersection >,
        StBase<StIntersectionTrafficLightStop>
{
  // on enter
  StIntersectionTrafficLightStop(my_context ctx);
  // on exit
  ~StIntersectionTrafficLightStop();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;

  // internal variables
};

} // namespace vlr

#endif // AW_STINTERSECTIONTRAFFICLIGHTSTOP_HPP_
