/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_STLANECHANGETYPES_HPP
#define AW_STLANECHANGETYPES_HPP


namespace vlr {

enum StLaneChangeLaneChangeType {
	LC_LEFT,
	LC_RIGHT,
	LC_LEFT_OPPOSITE,
	LC_IMPOSSIBLE,
	LC_NO_CHANGE
};

enum StLaneChangeLaneChangeReason {
	LC_CAUSE_OF_ROUTE,
	LC_CAUSE_OF_MOVING_VEHICLE,
	LC_CAUSE_OF_OBSTACLE
};

enum StLaneChangeRecoverType {
	LC_RECOVER_TO_STARTPOINT,
	LC_RECOVER_TO_ENDPOINT,
};

} // namespace vlr

#endif // AW_STLANECHANGETYPES_HPP
