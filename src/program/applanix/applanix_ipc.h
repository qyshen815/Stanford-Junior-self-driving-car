#ifndef DGC_APPLANIX_IPC_H
#define DGC_APPLANIX_IPC_H

#include <applanix_messages.h>

void 
dgc_applanix_register_ipc_messages(void);

void 
dgc_applanix_publish_pose_message(dgc::ApplanixPose *pose);

void 
dgc_applanix_publish_rms_message(dgc::ApplanixRms *rms);

void
dgc_applanix_publish_gps_message(dgc::ApplanixGps *gps);

void
dgc_applanix_publish_dmi_message(dgc::ApplanixDmi *dmi);

#endif
