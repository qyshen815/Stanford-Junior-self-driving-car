#ifndef AW_STINTERSECTIONTRAFFICLIGHTWAIT_HPP_
#define AW_STINTERSECTIONTRAFFICLIGHTWAIT_HPP_

#include "aw_StBase.hpp"
#include "aw_StActive.hpp"
#include "aw_StIntersection.hpp"

namespace vlr {

/*---------------------------------------------------------------------------
 * StIntersectionTrafficLightWait
 *---------------------------------------------------------------------------*/
struct StIntersectionTrafficLightWait:
        sc::state< StIntersectionTrafficLightWait,StIntersection>,
        StBase<StIntersectionTrafficLightWait>
{
  // on enter
  StIntersectionTrafficLightWait(my_context ctx);
  // on exit
  ~StIntersectionTrafficLightWait();

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

#endif // AW_STINTERSECTIONTRAFFICLIGHTWAIT_HPP_
