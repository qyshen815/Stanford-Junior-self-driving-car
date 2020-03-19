#ifndef LADYBUG_INTERFACE_H
#define LADYBUG_INTERFACE_H

#define LADYBUG_VERSION_UNKNOWN   1
#define LADYBUG_VERSION_2         2
#define LADYBUG_VERSION_3         3

namespace dgc {

const int kLadybugMaxPacketSize = 4000000;

struct LadybugPacket {
  LadybugPacket();
  ~LadybugPacket();

  double timestamp;
  
  double smooth_x;
  double smooth_y;
  double smooth_z;
  double roll;
  double pitch;
  double yaw;

  unsigned short version;
  unsigned int len, max_len;
  unsigned char *data;
};

class LadybugInterface {
 public:
  virtual ~LadybugInterface() {};
  virtual int CreateServer() = 0;
  virtual int CreateClient() = 0;

  virtual int ReadPacket(LadybugPacket *pkt) = 0;
  virtual int ReadCurrentPacket(LadybugPacket *pkt) = 0;
  virtual int WritePacket(LadybugPacket *pkt) = 0;
  virtual int DataWaiting(void) = 0;
};

}

#endif
