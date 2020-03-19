/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_STACTIVE_HPP
#define AW_STACTIVE_HPP

#include "aw_StBase.hpp"
#include "aw_ChsmPlanner.hpp"

namespace vlr {

struct StPause;
struct StDrive;

///! history pseudo state
typedef sc::deep_history< StDrive > StActiveHistory;
struct StActive: sc::state< StActive, ChsmPlanner, StDrive, sc::has_deep_history >, StBase< StActive >
{
  // on enter
  StActive(my_context ctx);
  // on exit
  virtual ~StActive();
  // reactions
  sc::result react(const EvProcess& evt);
  sc::result react(const EvAfterProcess& evt);
  sc::result react(const sc::exception_thrown& evt);

  typedef mpl::list
  <
    sc::custom_reaction< EvProcess >,
    sc::transition< EvPause, StPause >,
    sc::custom_reaction< sc::exception_thrown>
  > reactions;
};

// if state-machine is errournous this state will come to the rescue
struct StGlobalRecover: sc::state< StGlobalRecover, StActive >, StBase< StGlobalRecover >
{
  // on enter
	StGlobalRecover(my_context ctx);
  // on exit
  virtual ~StGlobalRecover();
  // reactions
  sc::result react(const EvAfterProcess& evt);

  typedef mpl::list
  <
    sc::custom_reaction< EvAfterProcess >
  > reactions;

  bool done;
  RndfVertex* checkpoint;
};

} // namespace vlr

#include "aw_StDrive.hpp"
#include "aw_StStop.hpp"
#include "aw_StIntersection.hpp"
#include "aw_StZone.hpp"

#endif // AW_STACTIVE_HPP
