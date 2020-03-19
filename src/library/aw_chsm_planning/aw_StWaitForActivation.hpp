/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_STWAITFORACTIVATION_HPP
#define AW_STWAITFORACTIVATION_HPP


#include "aw_StBase.hpp"
#include "aw_ChsmPlanner.hpp"
#include "aw_StPause.hpp"

namespace vlr {

struct StActive;
struct StWaitForActivation: public sc::state< StWaitForActivation, ChsmPlanner >, public StBase< StWaitForActivation >
{
  // on enter
  StWaitForActivation(my_context ctx);
  // on exit
  ~StWaitForActivation();
  // reaction
  sc::result react(const EvProcess& evt);
  sc::result react(const sc::exception_thrown& evt);

  // reactions
  typedef mpl::list
  <
    sc::custom_reaction< EvProcess >,
    sc::transition< EvPause, StPause >,
    sc::custom_reaction< sc::exception_thrown>
  > reactions;

  Timestamp waitUntil;
  bool moved;
};

} // namespace vlr

#endif // AW_STWAITFORACTIVATION_HPP
