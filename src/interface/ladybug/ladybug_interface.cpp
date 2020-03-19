#include <roadrunner.h>
#include <ladybug_interface.h>

namespace dgc {

LadybugPacket::LadybugPacket()
{
  len = 0;
  max_len = kLadybugMaxPacketSize;
  data = (unsigned char*)malloc(max_len);
}

LadybugPacket::~LadybugPacket()
{
  if(data != NULL)
    free(data);
}

}
