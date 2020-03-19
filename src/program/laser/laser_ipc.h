#ifndef LASER_IPC_H
#define LASER_IPC_H

#include "sick.h"

void dgc_laser_register_ipc_messages(void);

void publish_laser_heartbeat(int laser_num);

void publish_laser_heartbeat_old(int front_stalled, int rear_stalled,
                                 int laser3_stalled, int laser4_stalled);

void publish_laser_message(sick_laser_p laser);

#endif
