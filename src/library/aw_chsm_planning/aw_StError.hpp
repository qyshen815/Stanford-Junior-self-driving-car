/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_STERROR_HPP
#define AW_STERROR_HPP

#include "aw_StBase.hpp"
#include "aw_ChsmPlanner.hpp"

namespace vlr {

struct StError: public sc::state< StError, ChsmPlanner >, public StBase< StError >
{
  // on enter
  StError(my_context ctx);
  // on exit
  ~StError();
  // reaction
  sc::result react(const EvProcess& evt);

  // reactions
  typedef mpl::list
  <
    sc::custom_reaction< EvProcess >
  > reactions;
};

} // namespace vlr

#endif // AW_STERROR_HPP
