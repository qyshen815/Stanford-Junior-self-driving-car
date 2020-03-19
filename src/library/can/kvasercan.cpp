#include <roadrunner.h>
#include "kvasercan.h"
#include <stdint.h>

namespace dgc {

KvaserCan::KvaserCan(int channel, int baud)
{
  int numChannels, ret;
  ret = canGetNumberOfChannels(&numChannels);
  if(ret < 0) {
    dgc_die("Error getting number of active can channels");
  }
  if(numChannels == 0) {
    dgc_die("No can channels available on this machine");
  }
  if(channel < 0) {
    dgc_die("Requested a negative can channel.");
  }
  if(channel >= numChannels) {
    dgc_die("Requested can channel %d, but there are only %d available.",
        channel, numChannels);
  }
  channelNumber = channel;
  bus = -1;
  bitrate = baud;
}

KvaserCan::~KvaserCan()
{
  Close();
}

bool KvaserCan::Open()
{
  canStatus stat;
  unsigned long flags;

  bus = canOpenChannel(channelNumber, 0);//canWANT_EXCLUSIVE );
  if (bus < 0 ) {
    dgc_die("Could not open Kvaser can channel %d", channelNumber);
  }
  dgc_info("Opened Kvaser can channel %d", channelNumber);

  stat = canSetBusParams(bus, bitrate, 4,3,1,1,0);
  //stat = canSetBusParams(bus, bitrate, 0,0,0,0,0);
  if( !wasOkay(stat) ) {
    Close();
    return false;
  }
  // This may not be necessary (supposedly for bug in Kvaser Leaf boards)
  stat = canSetBusOutputControl(bus, canDRIVER_NORMAL);
  if( !wasOkay(stat) ) {
    Close();
    return false;
  }
  stat = canBusOn(bus);
  if( !wasOkay(stat) ) {
    Close();
    return false;
  }
  stat = canReadStatus(bus, &flags);
  if(flags)
    printf("STATUS --------\n");
  if(flags & canSTAT_ERROR_PASSIVE)
    printf("Error passive!\n");
  if(flags & canSTAT_BUS_OFF)
    printf("Bus off!\n");
  if(flags & canSTAT_ERROR_WARNING)
    printf("High error count!\n");
  if(flags & canSTAT_ERROR_ACTIVE)
    printf("Error active!\n");
  if(flags & canSTAT_TX_PENDING)
    printf("Tx Pending!\n");
  if(flags & canSTAT_RX_PENDING)
    printf("Rx Pending!\n");
  if(flags & canSTAT_TXERR)
    printf("Tx Error!\n");
  if(flags & canSTAT_RXERR)
    printf("Rx Error!\n");
  if(flags & canSTAT_HW_OVERRUN)
    printf("HW buffer overrun!\n");
  if(flags & canSTAT_SW_OVERRUN)
    printf("SW buffer overrun!\n");
  if(flags)
    printf("---------------\n");
  return true;
}

void KvaserCan::Close()
{
  canStatus stat;
  if(bus >= 0) {
    stat = canBusOff(bus);
    if(!wasOkay(stat))
      dgc_error("Trouble disabling can channel %d", channelNumber);
    stat = canClose(bus);
    if(!wasOkay(stat))
      dgc_error("Trouble closing can channel %d", channelNumber);
  }
  bus = -1;
}

int KvaserCan::ReadMessage(long *id, void* data)
{
  canStatus stat;
  uint8_t msg[8];
  uint32_t len;
  uint32_t flags;
  unsigned long time;

  if(bus < 0) {
    dgc_error("Cannot write to can channel %d because it is not open!", \
        channelNumber);
    return -1;
  }

  stat = canRead(bus, id, &msg, &len, &flags, &time);
  if( stat == canERR_NOMSG ) {
    return 0;
  }
  if( !wasOkay(stat) ) {
    return 0;
  }
  if( len < 1 || len > 8 ) {
    return 0;
    dgc_die("Fatal, can message received had length: %d", len);
  }
  memcpy(data, msg, len);
  return len; 
}

int KvaserCan::SendMessage(long id, void* data, int len)
{
  canStatus stat;
  unsigned long flags = 5;

  if(len < 1 || len > 8) {
    dgc_error("Requested can message be sent with length: %d", len);
    return 0;
  }

  /*stat = canReadStatus(bus, &flags);
  if(flags)
    printf("STATUS --------\n");
  if(flags & canSTAT_ERROR_PASSIVE)
    printf("Error passive!\n");
  if(flags & canSTAT_BUS_OFF)
    printf("Bus off!\n");
  if(flags & canSTAT_ERROR_WARNING)
    printf("High error count!\n");
  if(flags & canSTAT_ERROR_ACTIVE)
    printf("Error active!\n");
  if(flags & canSTAT_TX_PENDING)
    printf("Tx Pending!\n");
  if(flags & canSTAT_RX_PENDING)
    printf("Rx Pending!\n");
  if(flags & canSTAT_TXERR)
    printf("Tx Error!\n");
  if(flags & canSTAT_RXERR)
    printf("Rx Error!\n");
  if(flags & canSTAT_HW_OVERRUN)
    printf("HW buffer overrun!\n");
  if(flags & canSTAT_SW_OVERRUN)
    printf("SW buffer overrun!\n");
  if(flags)
    printf("---------------\n");
*/
  stat = canWrite(bus, id, data, len, 0);
  if( !wasOkay(stat) ) {
    return 0;
  } 
  //if( !wasOkay(canWriteSync(bus, 1000))) {
  //  return 0;
  //}
  return len;
}

bool KvaserCan::wasOkay(canStatus stat)
{
  char buf[100];
  if (stat != canOK) {
    buf[0] = '\0';
    canGetErrorText(stat, buf, sizeof(buf));
    dgc_error("Kvaser can command failed!\nStatus=%d (%s)", (int)stat, buf);
    return false;
  }
  return true;
}

}
