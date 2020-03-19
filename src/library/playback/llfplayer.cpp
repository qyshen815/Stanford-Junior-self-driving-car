#include <roadrunner.h>
#include "llfplayer.h"
#include <blf_id.h>

namespace dgc {

llf_player::llf_player(LadybugInterface *lint)
{
  lbug_interface = lint;
  blf = new blf_t;
  blf_index = NULL;
  pkt = NULL;
}

llf_player::~llf_player()
{
  if(pkt != NULL) {
    if(pkt->data != NULL)
      delete pkt->data;
    delete pkt;
  }
  delete lbug_interface;
  delete blf;
}

int llf_player::initialize(char *llf_filename)
{
  pkt = new LadybugPacket;

  if(blf->open(llf_filename, "r") != BLF_OK) {
    dgc_error("Could not open file %s for reading.", llf_filename);
    return -1;
  }
  blf_index = blf_index_load(llf_filename);
  if(blf_index == NULL) 
    dgc_warning("Could not open index file %s.idx.gz", llf_filename);
  return 0;
}

void llf_player::seek(double t)
{
  int i = 0;
  
  /* skip to right place */
  if(blf_index != NULL) {
    while(i < blf_index->num_blocks && 
	  blf_index->block[i].timestamp < t)
      i++;
    if(i >= blf_index->num_blocks)
      i = blf_index->num_blocks - 1;
    blf->seek(blf_index->block[i].offset, SEEK_SET);
  }
  else 
    blf->seek_timestamp(t);

  set_last_packet_time(t);
}

void llf_player::read_packet(double t, dgc_pose_p pose, double max_age)
{
  unsigned short pkt_id;
  unsigned int pkt_len;
  double pkt_timestamp;
  int ret;

  /* get the packet info without reading the packet */
  ret = blf->start_partial_read(&pkt_id, &pkt_timestamp, &pkt_len);
  if(ret == BLF_EOF) 
    set_eof(true);
  if(ret != BLF_OK) 
    return;

  /* skip it if it isn't a ladybug packet, or it is too old */
  if((pkt_id != BLF_LADYBUG2_ID &&
      pkt_id != BLF_LADYBUG3_ID &&
      pkt_id != BLF_UNKNOWN_ID) ||
     fabs(pkt_timestamp - t) > max_age) {
    set_last_packet_time(pkt_timestamp);
    blf->abort_partial_read();
    return;
  }
  /* skip if it is too large */
  else if(pkt_len > pkt->max_len) {
    dgc_error("Ladybug packet is too large.");
    set_last_packet_time(pkt_timestamp);
    blf->abort_partial_read();
    return;
  }

  /* read the ladybug data */
  ret = blf->partial_read(pkt->data, pkt_len);
  if(ret != BLF_OK) {
    blf->abort_partial_read();
    return;
  }
  pkt->len = pkt_len;

  /* finish reading packet, make sure chksum is ok */
  ret = blf->finish_partial_read();
  if(ret != BLF_OK)
    return;

  if (pkt_id==BLF_LADYBUG2_ID) {
    pkt->version = LADYBUG_VERSION_2;
  } else if (pkt_id==BLF_LADYBUG3_ID) {
    pkt->version = LADYBUG_VERSION_3;
  } else {
    pkt->version = LADYBUG_VERSION_UNKNOWN;
  }
  
  pkt->smooth_x = pose->x;
  pkt->smooth_y = pose->y;
  pkt->smooth_z = pose->z;
  pkt->yaw      = pose->yaw;
  pkt->pitch    = pose->pitch;
  pkt->roll     = pose->roll;
  pkt->timestamp = pkt_timestamp;
  lbug_interface->WritePacket(pkt);
  add_output_bytes(pkt_len);
  add_frames(1);
  set_last_packet_time(pkt_timestamp);
  add_input_bytes(pkt_len);
}

}
