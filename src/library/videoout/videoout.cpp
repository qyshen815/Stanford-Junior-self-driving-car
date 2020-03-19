#include <cmath>
#include <roadrunner.h>
#include <GL/glut.h>

extern "C" {
#include <videoout.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

int    video_recording  = FALSE;
long   video_frame_nr   = 0;
float  video_frame_rate = 0.0;
double video_start_time = 0.0;

pthread_mutex_t    vo_mutex  = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t     vo_cond   = PTHREAD_COND_INITIALIZER;

struct SwsContext  *video_sws = NULL;

/* add a video output stream */
AVStream *
add_video_stream( AVFormatContext *oc, CodecID codec_id, int bit_rate, 
		  int width, int height, int frame_rate, PixelFormat encoding_pix_fmt )
{
  AVCodecContext *c;
  AVStream *st;

  st = av_new_stream(oc, 0);
  if (!st) {
    fprintf(stderr, "Could not alloc stream\n");
    exit(1);
  }
    
  c = st->codec;
  c->codec_id = codec_id;
  c->codec_type = CODEC_TYPE_VIDEO;

  /* put sample parameters */
  c->bit_rate = bit_rate;
  /* resolution must be a multiple of two */
  c->width = width;  
  c->height = height;
  /* frames per second */
  c->time_base.den = frame_rate;  
  c->time_base.num = 1;
  if (encoding_pix_fmt != 0)
    c->pix_fmt = encoding_pix_fmt;
  else
    c->pix_fmt = PIX_FMT_YUV420P;
  c->strict_std_compliance=-1;
  c->gop_size = 12; /* emit one intra frame every twelve frames at most */
  if (c->codec_id == CODEC_ID_MPEG2VIDEO) {
    /* just for testing, we also add B frames */
    c->max_b_frames = 2;
  }
  if (c->codec_id == CODEC_ID_MPEG1VIDEO){
    /* needed to avoid using macroblocks in which some coeffs overflow 
       this doesnt happen with normal video, it just happens here as the 
       motion of the chroma plane doesnt match the luma plane */
    c->mb_decision=2;
  }
  // some formats want stream headers to be seperate
  if(!strcmp(oc->oformat->name, "mp4") || 
     !strcmp(oc->oformat->name, "mov") || 
     !strcmp(oc->oformat->name, "3gp"))
    c->flags |= CODEC_FLAG_GLOBAL_HEADER;
  
  return st;
}

AVFrame * 
alloc_picture( PixelFormat pix_fmt, int width, int height )
{
  AVFrame *picture;
  uint8_t *picture_buf;
  int size;
    
  picture = avcodec_alloc_frame();
  if (!picture) 
    return NULL;

  size = avpicture_get_size(pix_fmt, width, height);
  picture_buf = (uint8_t*)malloc(size);
  if (!picture_buf) {
    av_free(picture);
    return NULL;
  }
  avpicture_fill((AVPicture *)picture, picture_buf, 
                 pix_fmt, width, height);
  return picture;
}
    
void 
open_video( dgc_videoout_p vo, AVFormatContext *oc, AVStream *st )
{
  AVCodec *codec;
  AVCodecContext *c;

  c = st->codec;

  /* find the video encoder */
  codec = avcodec_find_encoder(c->codec_id);
  if (!codec) {
    fprintf(stderr, "codec not found\n");
    exit(1);
  }

  /* open the codec */
  if (avcodec_open(c, codec) < 0) {
    fprintf(stderr, "could not open video codec %d\n", c->codec_id);
    exit(1);
  }

  vo->video_outbuf = NULL;
  if (!(oc->oformat->flags & AVFMT_RAWPICTURE)) {
    /* allocate output buffer */
    /* XXX: API change will be done */
    vo->video_outbuf_size = 1400000;
    vo->video_outbuf = (uint8_t*)malloc(vo->video_outbuf_size);
  }

  /* allocate the encoded raw picture */
  vo->picture = alloc_picture(c->pix_fmt, c->width, c->height);
  if (!vo->picture) {
    fprintf(stderr, "Could not allocate picture\n");
    exit(1);
  }

}

/* prepare a dummy image */
void
fill_yuv_image( AVFrame *pict, int frame_index, int width, int height )
{
  int x, y, i;

  i = frame_index;

  /* Y */
  for(y=0;y<height;y++) {
    for(x=0;x<width;x++) {
      pict->data[0][y * pict->linesize[0] + x] = x + y + i * 3;
    }
  }
    
  /* Cb and Cr */
  for(y=0;y<height/2;y++) {
    for(x=0;x<width/2;x++) {
      pict->data[1][y * pict->linesize[1] + x] = 128 + y + i * 2;
      pict->data[2][y * pict->linesize[2] + x] = 64 + x + i * 5;
    }
  }
}


void
close_video( dgc_videoout_p vo, AVStream *st )
{
  avcodec_close(st->codec);
  av_free(vo->picture->data[0]);
  av_free(vo->picture);
  av_free(vo->video_outbuf);
}

dgc_videoout_p 
dgc_videoout_init( const char *filename, int bit_rate, 
		   int width, int height, int frame_rate, 
		   CodecID codec_id, unsigned int fourcc, PixelFormat encoding_pix_fmt )
{
  dgc_videoout_p vo = (dgc_videoout_p)malloc(sizeof(dgc_videoout_t));
  vo->frame_count = 0;
  vo->width = width;
  vo->height = height;
  vo->screenshot_buffer = NULL;
  vo->screenshot_buffer2 = NULL;
  
  /* initialize libavcodec, and register all codecs and formats */
  av_register_all();
    
  /* auto detect the output format from the name. default is
     mpeg. */
  vo->fmt = guess_format(NULL, filename, NULL);
  if (!vo->fmt) {
    printf("Could not deduce output format from file extension: using MPEG.\n");
    vo->fmt = guess_format("mpeg", NULL, NULL);
  }
  if (!vo->fmt) {
    fprintf(stderr, "Could not find suitable output format\n");
    exit(1);
  }
    
  /* allocate the output media context */
  vo->oc = avformat_alloc_context();
  if (!vo->oc) {
    fprintf(stderr, "Memory error\n");
    exit(1);
  }
  vo->oc->oformat = vo->fmt;
  snprintf(vo->oc->filename, sizeof(vo->oc->filename), "%s", filename);
        
  if (codec_id != CODEC_ID_NONE) {
    vo->fmt->video_codec = codec_id;        
    printf("Manually setting codec_id to %d\n", codec_id);
  }

  /* add the audio and video streams using the default format codecs
     and initialize the codecs */
  vo->video_st = NULL;
  if (vo->fmt->video_codec != CODEC_ID_NONE) {
    vo->video_st = add_video_stream(vo->oc, vo->fmt->video_codec, bit_rate, 
				    width, height, frame_rate, encoding_pix_fmt);
  }

  if (vo->video_st->codec->codec_id == CODEC_ID_MPEG4) {
    printf("setting fourcc codec code to DIVX\n");
    vo->video_st->codec->codec_tag = (('X' << 24) + ('V' << 16) + ('I' << 8) + 'D');
  }
  if (fourcc != 0)
    vo->video_st->codec->codec_tag = fourcc;

  /* set the output parameters (must be done even if no
     parameters). */
  if (av_set_parameters(vo->oc, NULL) < 0) {
    fprintf(stderr, "Invalid output format parameters\n");
    exit(1);
  }

  dump_format(vo->oc, 0, filename, 1);

  /* now that all the parameters are set, we can open the audio and
     video codecs and allocate the necessary encode buffers */
  if (vo->video_st)
    open_video(vo, vo->oc, vo->video_st);

  /* open the output file, if needed */
  if (!(vo->fmt->flags & AVFMT_NOFILE)) {
    if (url_fopen(&vo->oc->pb, filename, URL_WRONLY) < 0) {
      fprintf(stderr, "Could not open '%s'\n", filename);
      exit(1);
    }
  }
    
  /* write the stream header, if any */
  av_write_header(vo->oc);
  
  video_frame_rate = frame_rate;

  return vo;
}

int
dgc_videoout_add_frame(dgc_videoout_p vo, unsigned char *image_data, 
		       enum PixelFormat pix_fmt ) 
{
  int out_size, ret;
  AVCodecContext* c=NULL;
  AVFrame* picture_ptr=NULL;
  static int allocated = 0;
  static AVFrame* pic_in=NULL;
    
  c = vo->video_st->codec;
    
  if (!allocated) {
    pic_in = avcodec_alloc_frame();
    allocated = 1;
  }
  if (!pic_in || !image_data) {
    fprintf(stderr, "pic_in or image_data is NULL\n");
    return 0;
  }

  if (avpicture_fill((AVPicture *)pic_in, image_data, 
                     pix_fmt, c->width, c->height) < 0) {
    fprintf(stderr, "avpicture_fill failed\n");                           
    return 0;
  }                           
  
  if (pix_fmt != c->pix_fmt) {
    /* as we only accept a YUV420P picture, we must convert the given image
       into this if needed */

    video_sws = sws_getContext( c->width, c->height, pix_fmt, 
				c->width, c->height, c->pix_fmt, 
//        vo->oc->oformat->flags,
        SWS_BILINEAR,
        //        vo->oc->oformat->flags | SWS_BILINEAR,
				NULL, NULL, NULL);
    if(!video_sws) {
	fprintf(stderr, "failed to get scaler context.\n");                           
    	return 0;
	}
    sws_scale(video_sws, pic_in->data, pic_in->linesize, 0, c->height,
	      vo->picture->data, vo->picture->linesize ); 

    picture_ptr = vo->picture;
  } else {
    picture_ptr = pic_in;
  }
    
  if (vo->oc->oformat->flags & AVFMT_RAWPICTURE) {
    /* raw video case. The API will change slightly in the near
       futur for that */
    AVPacket pkt;
    av_init_packet(&pkt);
        
    pkt.flags |= PKT_FLAG_KEY;
    pkt.stream_index= vo->video_st->index;
    pkt.data= (uint8_t *)picture_ptr;
    pkt.size= sizeof(AVPicture);
        
    printf("written raw frame\n");
    ret = av_write_frame(vo->oc, &pkt);
  } else {
    /* encode the image */
    out_size = avcodec_encode_video(c, vo->video_outbuf, vo->video_outbuf_size, picture_ptr);
    /* if zero size, it means the image was buffered */
    if (out_size != 0) {
      AVPacket pkt;
      av_init_packet(&pkt);
            
      pkt.pts= c->coded_frame->pts;
      if(c->coded_frame->key_frame)
        pkt.flags |= PKT_FLAG_KEY;
      pkt.stream_index= vo->video_st->index;
      pkt.data= vo->video_outbuf;
      pkt.size= out_size;
            
      /* write the compressed frame in the media file */
      ret = av_write_frame(vo->oc, &pkt);
    } else {
      ret = 0;
    }
  }
  if (ret != 0) {
    fprintf(stderr, "Error while writing video frame\n");
    exit(1);
  }
  vo->frame_count++;
    
  //  av_free(pic_in);
  return ret;
}

unsigned char * 
dgc_videoout_flip24_vertically(unsigned char *src, int width, int height, 
			       int byte_per_pixel, unsigned char *dst ) {
  int i, j, ni, no; 
  int i_width_step = width * byte_per_pixel;
  int o_width_step = width * 3;
  if( dst == NULL )
    dst = (unsigned char *)malloc( height * o_width_step);
 
  for (i = 0; i < height; i++) {
    for( j = 0; j < width; j++) {
      ni = i * i_width_step + j * byte_per_pixel;
      no = (height-i-1) * o_width_step + j * 3;
      dst[no+0] = src[ni+0];
      dst[no+1] = src[ni+1];
      dst[no+2] = src[ni+2];
    }
  }
  return dst;        
}

unsigned char * 
dgc_videoout_flip_vertically(unsigned char *src, int width, int height, 
			     int byte_per_pixel, unsigned char *dst ) 
{
  int i, width_step = width * byte_per_pixel;
  unsigned char *y_src, *y_dst;
  if( dst == NULL )
    dst = (unsigned char *)malloc( height * width_step);
  
  y_src = src + (height-1) * width_step;
  y_dst = dst + 0;
  for (i = 0; i < height; i++) {
    memcpy( y_dst, y_src, 3*width );
    y_src -= width_step;
    y_dst += width_step;
  }
  return dst;        
}

void 
dgc_videoout_add_opengl_frame( dgc_videoout_p vo ) 
{
  static int allocated = 0;
  static char    *buf = NULL;
  static char    *buf2 = NULL;
  static GLenum   format;
  static int      bpp;
  long   frame_nr = 0;

  if( !allocated ) {
    bpp = glutGet(GLUT_WINDOW_RGBA) ? 4 : 3;
    format  =  (bpp==4) ? GL_RGBA : GL_RGB;
    vo->screenshot_buffer = (unsigned char *)realloc(buf, vo->width * vo->height * bpp);
    dgc_test_alloc(vo->screenshot_buffer);
    vo->screenshot_buffer2 = (unsigned char *)realloc(buf2, vo->width * vo->height * 3);
    dgc_test_alloc(vo->screenshot_buffer2);
    allocated = 1;
  }

  glReadPixels(0, 0, vo->width, vo->height,
	      format, GL_UNSIGNED_BYTE, vo->screenshot_buffer);

  if (bpp==4)
    dgc_videoout_flip24_vertically(vo->screenshot_buffer, 
				   vo->width, vo->height, bpp,
				   vo->screenshot_buffer2);
  else
    dgc_videoout_flip_vertically(vo->screenshot_buffer, 
				 vo->width, vo->height, bpp,
				 vo->screenshot_buffer2);
  
  if (!video_recording) {
    video_start_time = dgc_get_time();
    video_recording = TRUE;
  }
  frame_nr = 1+((dgc_get_time()-video_start_time)*video_frame_rate);
  while(video_frame_nr<frame_nr) {
    dgc_videoout_add_frame(vo, vo->screenshot_buffer2, PIX_FMT_RGB24);
    video_frame_nr++;
  }

}


void 
dgc_videoout_release( dgc_videoout_p *vo ) {
  int i;
  if ((*vo) == NULL)
    return;
  
  if( (*vo)->screenshot_buffer != NULL )
    free( (*vo)->screenshot_buffer );
  if( (*vo)->screenshot_buffer2 != NULL )
    free( (*vo)->screenshot_buffer2 );
        
  /* close each codec */
  if ((*vo)->video_st)
    close_video((*vo), (*vo)->video_st);

  /* write the trailer, if any */
  av_write_trailer((*vo)->oc);
    
  /* free the streams */
  for(i = 0; i < (signed)(*vo)->oc->nb_streams; i++) {
    av_freep(&(*vo)->oc->streams[i]);
  }

  if (!((*vo)->fmt->flags & AVFMT_NOFILE)) {
    /* close the output file */
#if LIBAVFORMAT_VERSION_INT < (51<<16 | 12<<8 | 1) ||  \
    LIBAVFORMAT_VERSION_INT >=(52<<16 | 7<<8 | 0)
    url_fclose((*vo)->oc->pb);
#else 
    url_fclose(&(*vo)->oc->pb);
#endif
  }

  /* free the stream */
  av_free((*vo)->oc);
  free(*vo);
  *vo = NULL;

  video_recording  = FALSE;
  video_frame_nr   = 0;
}

void *
vo_thread( void * vo_ptr )
{
  dgc_videoout_p   vo = (dgc_videoout_p) vo_ptr;
  long             frame_nr = 0;
  int              bpp = glutGet(GLUT_WINDOW_RGBA) ? 4 : 3;
  char            *buf = NULL, *buf2 = NULL;
  double           current_time, last_time, delta_s;
  int              frame_ctr = 0;

  vo->screenshot_buffer = (unsigned char *)realloc(buf, vo->width * vo->height * bpp);
  dgc_test_alloc(vo->screenshot_buffer);
  vo->screenshot_buffer2 = (unsigned char *)realloc(buf2, vo->width * vo->height * 3);
  dgc_test_alloc(vo->screenshot_buffer2);

  vo->mt_loop = 1;
  last_time = current_time = dgc_get_time();

  do {
    pthread_mutex_lock(&vo_mutex);
    pthread_cond_wait( &vo_cond, &vo_mutex );
    if (!vo->mt_loop) 
      break;
    else {
      if (bpp==4)
	dgc_videoout_flip24_vertically(vo->screenshot_buffer, 
				       vo->width, vo->height, bpp,
				       vo->screenshot_buffer2);
      else
	dgc_videoout_flip_vertically(vo->screenshot_buffer, 
				     vo->width, vo->height, bpp,
				     vo->screenshot_buffer2);
      if (!video_recording) {
	video_start_time = dgc_get_time();
	video_recording = TRUE;
      }
      frame_nr = 1+((dgc_get_time()-video_start_time)*video_frame_rate);
      while(video_frame_nr<frame_nr) {
	  dgc_videoout_add_frame(vo, vo->screenshot_buffer2, PIX_FMT_RGB24);
	  video_frame_nr++;
      }
      current_time = dgc_get_time();
      delta_s = current_time - last_time;
      frame_ctr++;
      if (delta_s>1.0) {
	fprintf( stderr, "# INFO: write %.2f fps\n", frame_ctr / delta_s );
	frame_ctr = 0;
	last_time = current_time;
      }
    }
    pthread_mutex_unlock(&vo_mutex);
  } while (vo->mt_loop);
  dgc_videoout_release( &vo );
  pthread_mutex_unlock(&vo_mutex);
  return NULL;
}

dgc_videoout_p 
dgc_videoout_init_mt( const char *filename, int bit_rate, 
		      int width, int height, int frame_rate, 
		      CodecID codec_id, unsigned int fourcc, PixelFormat encoding_pix_fmt )
{
  pthread_t        t_thread;
  dgc_videoout_p   vo;

  vo = dgc_videoout_init( filename, bit_rate, width, height, 
			  frame_rate, codec_id, fourcc, 
			  encoding_pix_fmt );

  if (pthread_create( &t_thread, NULL, vo_thread, (void *)vo ) != 0) {
    fprintf( stderr, "# ERROR: can't start videoout thread\n" );
    exit(0);
  }
  return(vo);
}

void 
dgc_videoout_release_mt( dgc_videoout_p *vo )
{
  (*vo)->mt_loop = 0; 
  pthread_cond_signal(&vo_cond);
}

void 
dgc_videoout_add_opengl_frame_mt( dgc_videoout_p vo ) 
{
  GLenum           format = glutGet(GLUT_WINDOW_RGBA) ? GL_RGBA : GL_RGB;

  pthread_mutex_lock(&vo_mutex);
  glReadPixels(0, 0, vo->width, vo->height,
	      format, GL_UNSIGNED_BYTE, vo->screenshot_buffer);
  pthread_mutex_unlock(&vo_mutex);
  pthread_cond_signal(&vo_cond);

}

