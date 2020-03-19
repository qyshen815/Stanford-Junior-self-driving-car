#ifndef VLR_KVASERCAN_H
#define VLR_KVASERUSBCAN_H

#include "can.h"
#include <canlib.h>

namespace dgc {

class KvaserCan {
  public:
    KvaserCan(int, int);
    ~KvaserCan();

    virtual bool Open();
    virtual void Close();
    virtual int SendMessage(long, void *, int);
    virtual int ReadMessage(long *, void *);

  private:

    bool wasOkay(canStatus stat);
    canHandle bus;
    int channelNumber;
    int bitrate;
};

}

#endif //VLR_KVASER_CAN_H
