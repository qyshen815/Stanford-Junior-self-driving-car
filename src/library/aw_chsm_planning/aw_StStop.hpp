/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_STSTOP_HPP
#define AW_STSTOP_HPP

#include "aw_StBase.hpp"
#include "aw_StActive.hpp"

namespace vlr {

/*---------------------------------------------------------------------------
 * StStop
 *---------------------------------------------------------------------------*/
struct StStopping;
struct StStop : sc::state< StStop, StActive , StStopping>, StBase<StStop> {
  // on enter
  StStop(my_context ctx);
  // on exit
  ~StStop();

  // reactions
  sc::result react(const EvProcess& evt);
  sc::result react(const EvDrive& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >,
  sc::custom_reaction< EvDrive >
  > reactions;
};

/*---------------------------------------------------------------------------
 * StStopping
 *---------------------------------------------------------------------------*/
struct StStopping : sc::state< StStopping, StStop >, StBase<StStopping>
{
  // on enter
  StStopping(my_context ctx);
  // on exit
  ~StStopping();

  // reactions
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
  sc::custom_reaction< EvProcess >
  > reactions;
};

} // namespace vlr

#endif // AW_STSTOP_HPP
