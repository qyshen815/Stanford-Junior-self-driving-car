/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: 10/24/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef CHSM_PLANNING_RTDB_H_
#define CHSM_PLANNING_RTDB_H_

#include <kogmo.h>
#include <kogmo_rtdb.h>

#define KOGMO_RTDB_OBJTYPE_CHSM_FEEDBACK                   0xAA00E1
#define KOGMO_RTDB_OBJNAME_CHSM_FEEDBACK                   "chsm_feedback"
#define KOGMO_RTDB_HISTORY_INTERVAL_CHSM_FEEDBACK          0.5
#define KOGMO_RTDB_AVG_CYCLETIME_CHSM_FEEDBACK             0.1
#define KOGMO_RTDB_MAX_CYCLETIME_CHSM_FEEDBACK             0.2
#define KOGMO_RTDB_OBJTYPE_CHSM_STATE                      0xAA00E2
#define KOGMO_RTDB_CHSM_STATE_MAXSTRING                    255

#ifdef __cplusplus
 extern "C" {
 namespace KogniMobil {
#endif

typedef struct {
  kogmo_rtdb_subobj_base_t base;
  char current_edge_name[255];
  dgc_pose_t current_mission_offset;
} kogmo_rtdb_obj_chsm_feedback_t, *kogmo_rtdb_obj_chsm_feedback_p;

typedef struct {
  kogmo_rtdb_subobj_base_t base;
  char message[KOGMO_RTDB_CHSM_STATE_MAXSTRING+1];
} kogmo_rtdb_obj_chsm_state_t, *kogmo_rtdb_obj_chsm_state_p;


#ifdef __cplusplus
}; /* namespace KogniMobil */
}; /* extern "C" */
#endif

#endif /*CHSM_PLANNING_RTDB_H_*/
