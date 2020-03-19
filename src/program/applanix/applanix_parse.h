#ifndef DGC_APPLANIX_PARSE_H
#define DGC_APPLANIX_PARSE_H

#include <applanix_messages.h>

#define      APPLANIX_PARSE_ERROR         (-1)
#define      APPLANIX_PARSE_UNFINISHED    (0)

int 
applanix_parse_generic_message(char *buffer, int buffer_length);

int
applanix_parse_pose_message(char *buffer, int buffer_len, 
                            dgc::ApplanixPose *pose);
int
applanix_parse_rms_message(char *buffer, int buffer_len, 
                           dgc::ApplanixRms *rms);

int
applanix_parse_gps_message(char *buffer, int buffer_len, int *sats);

int
applanix_parse_time_message(char *buffer, int buffer_length, int *sync_mode);

int
applanix_parse_gams_message(char *buffer, int buffer_len, int *code);

int
applanix_parse_dmi_message(char *buffer, int buffer_length, 
                           dgc::ApplanixDmi *dmi);

#endif
