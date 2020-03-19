#ifndef DGC_VIDEOOUT_H
#define DGC_VIDEOOUT_H

#define attribute_deprecated
#define __STDC_CONSTANT_MACROS // Must show up before avformat.h for definition of UINT64_C on Ubuntu 11.04.
#include <libavformat/avformat.h>


#ifdef HAVE_VIDEOOUT

extern "C" {
  typedef struct {
    AVOutputFormat *fmt;
    AVFormatContext *oc;
    AVStream *video_st;
    AVFrame *picture;
  
    uint8_t *video_outbuf;
    int frame_count, video_outbuf_size;
    int width, height;
    unsigned char *screenshot_buffer, *screenshot_buffer2;

    int mt_loop;
  } dgc_videoout_t, *dgc_videoout_p;

  dgc_videoout_p dgc_videoout_init( const char *filename, 
				    int bit_rate, int width, int height, int frame_rate, 
				    CodecID codec_id, unsigned int fourcc, PixelFormat encoding_pix_fmt );

  dgc_videoout_p dgc_videoout_init_mt( const char *filename, 
				       int bit_rate, int width, int height, 
				       int frame_rate, CodecID codec_id, 
				       unsigned int fourcc, PixelFormat encoding_pix_fmt );

  int dgc_videoout_add_frame( dgc_videoout_p vo, unsigned char *image_data,
			      enum PixelFormat pix_fmt );

  void dgc_videoout_release( dgc_videoout_p *vo );

  void dgc_videoout_release_mt( dgc_videoout_p *vo );

  void dgc_videoout_add_opengl_frame( dgc_videoout_p vo );

  void dgc_videoout_add_opengl_frame_mt( dgc_videoout_p vo );

  unsigned char * 
  dgc_videoout_flip24_vertically(unsigned char *src, int width, int height, 
				 int byte_per_pixel, unsigned char *dst);

  unsigned char * 
  dgc_videoout_flip_vertically(unsigned char *src, int width, int height, 
			       int byte_per_pixel, unsigned char *dst);
  
}
#else

  typedef int dgc_videoout_t, *dgc_videoout_p;

#endif

#endif
