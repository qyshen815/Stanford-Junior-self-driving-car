#ifndef LADYBUG_PLAYBACK_H
#define LADYBUG_PLAYBACK_H

#include <blf.h>
#include <blf_id.h>
#include <roadrunner.h>
#include <ladybug_interface.h>
#include <ladybug_frameparser.h>
#include <fastjpeg.h>
#include <stdint.h>
#include <image.h>

#define LB3_width     1616
#define LB3_height    1232

class LadybugPlayback {

  protected:

    jpeg_decompress_struct    *fj[4];
    dgc::LadybugPacket   *pkt;
    blf_t           *blf;
    blf_index_t     *blf_index;

  public:

    LadybugPlayback();
    void          openLLF(char* blf_file);
    bool          readTimestampPacket(double time);
    bool          readNextPacket();
    /* Returns the timestamp of the current packet, or 0 if none. */
    double        getTimestamp();
    /* Pointer this returns is temporary.  The memory will be clobbered
     * by the next call to cameraImage, so make the most of the time you
     * have with your dgc_image_t
     *
     * Returns the specified camera image contained in the last Packet read.
     */
    vlr::dgc_image_t*  cameraImage(int cam_num);      

  private:

    vlr::dgc_image_t camImage;
    vlr::dgc_image_t fullBayerImage;
    vlr::dgc_image_t bayerImage[4];
};

#endif //LADYBUG_PLAYBACK_H
