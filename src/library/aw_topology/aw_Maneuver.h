#ifndef AW_MANEUVER_H
#define AW_MANEUVER_H

namespace vlr {

enum maneuver_t
{
  UC_MANEUVER_START_MISSION, //!< before mission start
  UC_MANEUVER_TRAVEL, //!< normal travel
  UC_MANEUVER_STOP_SIGN, //!< driving towards a stop sign
  UC_MANEUVER_CROSSWALK, //!< driving towards a crosswalk
  UC_MANEUVER_TRAFFIC_LIGHT, //!< driving towards a traffic light
  UC_MANEUVER_INT_TURN_RIGHT, //!< turnoff at intersection required (apply indicator!)
  UC_MANEUVER_INT_TURN_LEFT,
  UC_MANEUVER_INT_STRAIGHT,
  UC_MANEUVER_CURVE_RIGHT, //!< curves without intersection
  UC_MANEUVER_CURVE_LEFT,
  UC_MANEUVER_LANECHANGE_RIGHT, //!< lane changes
  UC_MANEUVER_LANECHANGE_LEFT,
  UC_MANEUVER_U_TURN, //!< U turn
  UC_MANEUVER_NAVIGATE, //!< free navigation behavior in parking zones
  UC_MANEUVER_PARKING, //!< parking spot
  UC_MANEUVER_CHECKPOINT, //!< other checkpoint (cross precisely!)
  UC_K_TURN,        //!< Dreipunktwende
  UC_MANEUVER_GOAL_REACHED, //!< goal checkpoint has been reached, stop
  UC_MANEUVER_ZONE_ENTRY, //!< entering a zone
  UC_MANEUVER_ZONE_EXIT, //!< exiting a zone
};

enum area_type_t
{
  UC_TRAVEL_AREA,
  UC_ZONE,
  UC_PARKING_ZONE
};

} // namespace vlr

#endif // AW_MANEUVER_H
