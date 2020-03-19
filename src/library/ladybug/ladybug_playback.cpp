#include <ladybug_playback.h>
#include <blf.h>
#include <blf_id.h>
#include <roadrunner.h>
#include <ladybug_interface.h>
#include <ladybug_frameparser.h>
#include <fastjpeg.h>
#include <stdint.h>
#include <image.h>
#include <global.h>
#include <dc1394/conversions.h>

using namespace dgc;
using namespace vlr;


double LadybugPlayback::getTimestamp()
{
  return pkt->timestamp;
}

LadybugPlayback::LadybugPlayback()
{
  bayerImage[0].pix = NULL;
  bayerImage[1].pix = NULL;
  bayerImage[2].pix = NULL;
  bayerImage[3].pix = NULL;
  fullBayerImage.pix = NULL;
  camImage.pix = NULL;
}

void LadybugPlayback::openLLF( char* blf_file )
{

  blf = new blf_t;
  if(blf->open(blf_file, "r") != BLF_OK)
    dgc_fatal_error("Could not open BLF file %s for reading.", blf_file);

  blf_index = blf_index_load(blf_file);
  if(blf_index == NULL)
    dgc_fatal_error("Could not open BLF index file associated with %s.", blf_file);

  int seconds = blf_index->block[blf_index->num_blocks-1].timestamp -
                blf_index->block[0].timestamp;
  int hours = seconds / 3600;
  int min = (seconds - hours * 3600) / 60;
  int sec = seconds - hours * 3600 - min * 60;
  dgc_info( "Opened BLF file %s.", blf_file );
  dgc_info( "Logfile length: %02dh:%02dmin:%02dsec\n", hours, min, sec );

  pkt = new LadybugPacket;
  
  for(int i = 0; i < 4; i++) 
    fj[i] = dgc_fastjpeg_init_decompress();

  for(int i = 0; i < 4; i++) {
    bayerImage[i].width = LB3_width / 2;
    bayerImage[i].height = LB3_height / 2;
    bayerImage[i].nchannels = 1;
    if(bayerImage[i].pix)
      free(bayerImage[i].pix);
    bayerImage[i].pix = (uint8_t*)malloc(bayerImage[i].width * 
        bayerImage[i].height * bayerImage[i].nchannels);
  }
  if(fullBayerImage.pix)
    free(fullBayerImage.pix);
  if(camImage.pix)
    free(camImage.pix);
  // These are rotated, so h and w are flipped on purpose
  fullBayerImage.width = LB3_height;
  fullBayerImage.height = LB3_width;
  fullBayerImage.nchannels = 1;
  fullBayerImage.pix = (uint8_t*)malloc(fullBayerImage.width *
      fullBayerImage.height * fullBayerImage.nchannels);
  camImage.width = LB3_height;
  camImage.height = LB3_width;
  camImage.nchannels = 3;
  camImage.pix = (uint8_t*)malloc(camImage.width * camImage.height *
      camImage.nchannels);
}

bool LadybugPlayback::readTimestampPacket(double time)
{
  int ret;

  ret = blf->seek_timestamp(time);
  if( ret == BLF_EOF ) {
    dgc_warning( "Reached end of LLF file." );
    return false;
  }
  if( ret != BLF_OK ) {
    dgc_error( "Error seeking timestamp from LLF file." );
    return false;
  }

  //printf("Requested timestamp: %f\n", time);
  return readNextPacket();
}

bool LadybugPlayback::readNextPacket()
{
  int ret;
  uint16_t pkt_id;

  ret = blf->read_data( &pkt_id, &(pkt->timestamp), &(pkt->data), &(pkt->len),
      &(pkt->max_len) );
  if( ret == BLF_EOF ) {
    dgc_warning( "Reached end of LLF file." );
    return false;
  }
  if( ret != BLF_OK ) {
    dgc_error( "Error reading packet from LLF file." );
    return false;
  }

  pkt->version = LADYBUG_VERSION_3;
  //printf("Packet timestamp: %f\n", pkt->timestamp);
  return true;
}

/* Pointer this returns is temporary.  The memory will be clobbered
 * by the next call to cameraImage, so make the most of the time you
 * have with your dgc_image_t
 */
vlr::dgc_image_t*  LadybugPlayback::cameraImage(int cam_num)
{

  LadybugCompressorHeaderInfo     compressorInfo;
  double timer = 0.0;

  if( compressorInfo.parse(pkt->data, 0, 0) != LADYBUG_OK ) {
    dgc_error( "Could not read header data from packet." );
    //return NULL;
  }

  timer = dgc_get_time();
  for( int i = 0; i < 4; i ++) {
    const LadybugCompressorHeaderInfo::ImageInfo* pimageinfo = 
        compressorInfo.getInfo( cam_num * 4 + i );

    dgc_fastjpeg_decompress_memory( fj[i], 
        pimageinfo->pData,
        pimageinfo->size,
        &(bayerImage[i]),
        NULL );
  }
  printf("  Decompressing: %f\n", dgc_get_time() - timer);
  timer = dgc_get_time();

  int i = 0;
  for( int x = LB3_width / 2 - 1; x >= 0; x-- ) {
    for( int y = LB3_height / 2 - 1; y >= 0; y-- ) {
      fullBayerImage.pix[i] = bayerImage[3].pix[x + y * LB3_width / 2];
      fullBayerImage.pix[i+1] = bayerImage[1].pix[x + y * LB3_width / 2];
      fullBayerImage.pix[i + LB3_height] = 
        bayerImage[2].pix[x + y * LB3_width / 2];
      fullBayerImage.pix[i + LB3_height + 1] = 
        bayerImage[0].pix[x + y * LB3_width / 2];
      i += 2;
      if( (i / LB3_height) % 2 == 1 )
        i += LB3_height; 
    }
  }
  printf("  Interleaving: %f\n", dgc_get_time() - timer);
  timer = dgc_get_time();

  if (dc1394_bayer_decoding_8bit(fullBayerImage.pix, camImage.pix,
      LB3_height, LB3_width, DC1394_COLOR_FILTER_BGGR,
      DC1394_BAYER_METHOD_HQLINEAR) != DC1394_SUCCESS )
    dgc_error( "Error decoding bayer image in libdc1394" );
  printf("  Debayering: %f\n", dgc_get_time() - timer);

  return &(camImage); 

}  
