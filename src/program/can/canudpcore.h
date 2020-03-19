#ifndef DGC_CANUDPCORE_H
#define DGC_CANUDPCORE_H

#include <roadrunner.h>
#include <can_messages.h>
#include "udpsock.h"

namespace dgc {

class can_udp_connection : public udp_connection {
 public:
  dgc::CanStatus *read_status_message(double timeout);
 private:
  dgc::CanStatus can;
};

}

#endif
