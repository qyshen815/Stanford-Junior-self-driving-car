/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_STPAUSE_HPP
#define AW_STPAUSE_HPP

#include "aw_StBase.hpp"
#include "aw_ChsmPlanner.hpp"

namespace vlr {

struct StWaitForActivation;
struct StPauseShortTerm;
struct StPause: public sc::state< StPause, ChsmPlanner, StPauseShortTerm >, public StBase< StPause >
{
  // on enter
  StPause(my_context ctx);
  // on exit
  ~StPause();
  // reaction
  sc::result react(const EvProcess& evt);
  sc::result react(const sc::exception_thrown& evt);

  // reactions
  typedef mpl::list
  <
    sc::custom_reaction< EvProcess >,
    sc::transition< EvActivate, StWaitForActivation >,
    sc::custom_reaction< sc::exception_thrown>
  > reactions;

  //CurvePoints save_curvepoints;
};

struct StPauseShortTerm: public sc::state< StPauseShortTerm, StPause >, public StBase< StPauseShortTerm >
{
  // on enter
  StPauseShortTerm(my_context ctx);
  // on exit
  ~StPauseShortTerm();
  // reaction
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
    sc::custom_reaction< EvProcess >
  > reactions;

  Timestamp switchTime;
};

struct StPauseLongTerm: public sc::state< StPauseLongTerm, StPause >, public StBase< StPauseLongTerm >
{
  // on enter
  StPauseLongTerm(my_context ctx);
  // on exit
  ~StPauseLongTerm();
  // reaction
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
    sc::custom_reaction< EvProcess >
  > reactions;
};

} // namespace vlr

#endif // AW_STPAUSE_HPP
