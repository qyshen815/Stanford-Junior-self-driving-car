/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team vlr
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_RNDFTOKENS_H
#define AW_RNDFTOKENS_H

namespace vlr {

// General
#define RNDF_DELIMITER                          "\012 \015 \t \011"

  // Misc
#define SRNDF_HEADER                            "SRNDF"   // Header for Mike's SRNDFs (obsolete)
#define SRNDF_IDSTRING                          "id_string"
#define SRNDF_LIBVERSION                        "rndf_lib_version"

// Road Network
#define RNDF_ROADNETWORK_NAME                   "RNDF_name"
#define RNDF_ROADNETWORK_NUM_SEGMENTS           "num_segments"
#define RNDF_ROADNETWORK_NUM_ZONES              "num_zones"
#define RNDF_ROADNETWORK_NUM_INTERSECTIONS      "num_intersections"
#define RNDF_ROADNETWORK_FORMAT_VERSION         "format_version"
#define RNDF_ROADNETWORK_CREATION_DATE          "creation_date"
#define RNDF_ROADNETWORK_END_FILE               "end_file"

// Segments
#define RNDF_SEGMENT_BEGIN                      "segment"
#define RNDF_SEGMENT_END                        "end_segment"
#define RNDF_SEGMENT_NAME                       "segment_name"
#define RNDF_SEGMENT_NUM_LANES                  "num_lanes"
#define RNDF_SEGMENT_SPEED_LIMIT                "speed_limit"
#define RNDF_SEGMENT_NUM_CROSSWALKS             "num_crosswalks"
#define RNDF_SEGMENT_OFFROAD                    "offroad"

// Lanes
#define RNDF_LANE_BEGIN                         "lane"
#define RNDF_LANE_END                           "end_lane"
#define RNDF_LANE_NUM_WAYPOINTS                 "num_waypoints"
#define RNDF_LANE_WIDTH                         "lane_width"
#define RNDF_LANE_TYPE                          "lane_type"
#define RNDF_LANE_LEFT_BOUNDARY                 "left_boundary"
#define RNDF_LANE_RIGHT_BOUNDARY                "right_boundary"
#define RNDF_LANE_SPEED_LIMIT                   "speed_limit" // not used yet...

// Lane boundary
#define RNDF_LANE_BOUNDARYTYPE_SOLIDWHITE       "solid_white"
#define RNDF_LANE_BOUNDARYTYPE_BROKENWHITE      "broken_white"
#define RNDF_LANE_BOUNDARYTYPE_SOLIDYELLOW      "solid_yellow"
#define RNDF_LANE_BOUNDARYTYPE_DOUBLEYELLOW     "double_yellow"

// Lane type
#define RNDF_LANE_TYPE_CARLANE                  "car_lane"
#define RNDF_LANE_TYPE_BIKELANE                 "bike_lane"

// Checkpoints
#define RNDF_CHECKPOINT                         "checkpoint"
// Exits
#define RNDF_EXIT                               "exit"
// Stops
#define RNDF_STOP                               "stop"
// Crosswalks
#define RNDF_CROSS                              "cross"
// Traffic lights
#define RNDF_LIGHT                              "light"

// Zones
#define RNDF_ZONE_BEGIN                         "zone"
#define RNDF_ZONE_END                           "end_zone"
#define RNDF_ZONE_NUM_SPOTS                     "num_spots"
#define RNDF_ZONE_NAME                          "zone_name"
#define RNDF_ZONE_OFFROAD                       "offroad"

// Perimeter
#define RNDF_PERIMETER_BEGIN                     "perimeter"
#define RNDF_PERIMETER_END                       "end_perimeter"
#define RNDF_PERIMETER_NUM_PERIMETERPOINTS       "num_perimeterpoints"

// Spots
#define RNDF_SPOT_BEGIN                          "spot"
#define RNDF_SPOT_END                            "end_spot"
#define RNDF_SPOT_WIDTH                          "spot_width"

// Intersections
#define RNDF_INTERSECTION_BEGIN                  "intersection"
#define RNDF_INTERSECTION_END                    "end_intersection"
#define RNDF_INTERSECTION_NUM_LIGHTS             "num_trafficlights"

// Traffic lights
#define RNDF_TRAFFIC_LIGHT_BEGIN                 "trafficlight"
#define RNDF_TRAFFIC_LIGHT_END                   "end_trafficlight"
#define RNDF_TRAFFIC_LIGHT_GROUP_ID              "group_id"
#define RNDF_TRAFFIC_LIGHT_POSITION              "position"

// Crosswalks
#define RNDF_CROSSWALK_BEGIN                    "crosswalk"
#define RNDF_CROSSWALK_END                      "end_crosswalk"
#define RNDF_CROSSWALK_WIDTH                    "crosswalk_width"
#define RNDF_CROSSWALK_P1                       "crosswalk_p1"
#define RNDF_CROSSWALK_P2                       "crosswalk_p2"
#define RNDF_CROSSWALK_TYPE_STOP                "stop"
#define RNDF_CROSSWALK_TYPE_INCOMING            "incoming"

} // namespace vlr

#endif
