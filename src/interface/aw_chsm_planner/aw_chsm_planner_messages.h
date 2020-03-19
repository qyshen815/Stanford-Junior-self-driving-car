/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_CHSM_PLANNER_MESSAGES_H
#define AW_CHSM_PLANNER_MESSAGES_H

#include <global.h>
#include <ipc_interface.h>

namespace vlr {

#define KOGMO_RTDB_CHSM_STATE_MAXSTRING                    255

typedef struct {
  char current_edge_name[255+1];
  dgc_pose_t current_mission_offset;
} kogmo_rtdb_obj_chsm_feedback_t, *kogmo_rtdb_obj_chsm_feedback_p;

#define                AW_CHSM_PLANNER_FEEDBACK_NAME       "aw_chsm_planner_feedback"
#define                AW_CHSM_PLANNER_FEEDBACK_FMT        "{[char:256],[double:6]}"

const dgc::IpcMessageID AWChsmPlannerFeedbackID = {AW_CHSM_PLANNER_FEEDBACK_NAME, AW_CHSM_PLANNER_FEEDBACK_FMT};

typedef struct {
  char message[KOGMO_RTDB_CHSM_STATE_MAXSTRING+1];
} kogmo_rtdb_obj_chsm_state_t, *kogmo_rtdb_obj_chsm_state_p;

#define                AW_CHSM_PLANNER_STATE_NAME       "aw_chsm_planner_state"
#define                AW_CHSM_PLANNER_STATE_FMT        "{[char:256]}"

const dgc::IpcMessageID AWChsmPlannerStateID = {AW_CHSM_PLANNER_STATE_NAME, AW_CHSM_PLANNER_STATE_FMT};
}

#endif
