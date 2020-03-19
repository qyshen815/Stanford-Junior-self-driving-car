/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/

#ifndef AW_CHSMEVENTS_HPP
#define AW_CHSMEVENTS_HPP

#include <boost/statechart/event.hpp>
#include <boost/pool/pool_alloc.hpp>

namespace sc = boost::statechart;

namespace vlr {
//! signals the desire to go in pause mode.
/*!
 * This event is fired when the "Pause" button on the remote was pressed.
 */
struct EvPause : sc::event< EvPause, boost::pool_allocator < char > > {};

//! signals the desire to go in active mode.
/*!
 * This event is fired when the "Pause" button on the remote was released.
 */
struct EvActivate : sc::event< EvActivate, boost::pool_allocator < char > > {};

//! The "do work" event. Gets fired every cycle.
/*!
 * On reacting to this event, the states can do what they have to do (e.g. generate curvepoints).
 */
struct EvProcess : sc::event< EvProcess, boost::pool_allocator < char > > {};

//! Gets fired every cycle after EvProcess was fired.
/*!
 * With reacting on this event the states can alter and/or override
 * the output of the EvProcess phase. Safty checks can be implemented as reaction
 * on this event.
 * This event should bubble up to outer states.
 *
 * @remark states _should not_ call transit<>() as reaction on this special event.
 *         They should nearly always call forward_event() as their last reaction
 *         on this event to bubble it up to outer states so that this states also can
 *         react on the EvAfterProcess event.
 *
 * @see StActive::react(const EvAfterProcess& evt) for an example reaction on this event.
 */
struct EvAfterProcess : sc::event< EvAfterProcess, boost::pool_allocator < char > > {};

struct EvStop : sc::event< EvStop, boost::pool_allocator < char > > {};
struct EvDrive : sc::event< EvDrive, boost::pool_allocator < char > > {};
struct EvCloseToIntersection : sc::event< EvCloseToIntersection, boost::pool_allocator < char > > {};

} // namespace vlr

#endif /*CHSMEVENTS_HPP_*/
