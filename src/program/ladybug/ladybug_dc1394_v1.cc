#include <roadrunner.h>
#include <stdarg.h>
#include <dc1394/dc1394_control.h>
#include <dc1394/dc1394_register.h>

#include "ladybug_dc1394.h"

unsigned int ladybug_check_frame( unsigned char *ptr );

static double    gamma_correction          = LB_GAMMA_CORRECTION;
static int       exposure_value            = LB_EXPOSURE_VALUE;
static int       jpeg_current_compression  = LB_JPEG_COMPRESSION_FACTOR;
static int       jpeg_compression_modifier = 0;

static dc1394camera_t *camera = NULL;
static short dma_buffer_locked = 0;

namespace dgc {

ladybug_version_t  ladybug_version = UNKNOWN;

unsigned int 
ladybug_get_base_serial_no() 
{
  static uint val = (uint)-1;
  int err;

  if( val == (uint)-1 ) {
    err = GetCameraControlRegister( camera, 0x1f20, &val );
    if( err != DC1394_SUCCESS ) 
      dgc_die("Could not read serial number, err=%d, val=%d\n", err, val);
  }
  return val;
}

unsigned int 
ladybug_get_head_serial_no() 
{
  static uint val = (uint) -1;
  int err;

  if( val == (uint)-1 ) {
    err = GetCameraControlRegister( camera, 0x1f80, &val );
    if( err != DC1394_SUCCESS ) 
      dgc_die("Could not read serial number, err=%d, val=%d\n", err, val);
  }
  return val;
}

void
ladybug_print_image_brightness_v1() 
{
  uint val;
  if( GetCameraControlRegister( camera, 0x81c, &val ) != DC1394_SUCCESS ) 
    dgc_die("Could not get shutter speed factor #1\n");
  uint shutter = val >> 20; 

  if( GetCameraControlRegister( camera, 0x800, &val ) != DC1394_SUCCESS ) 
    dgc_die("Could not get brightness\n");
  uint brightness = val >> 20; 

  if( GetCameraControlRegister( camera, 0x804, &val ) != DC1394_SUCCESS ) 
    dgc_die("Could not get exposure\n");
  uint exposure = val >> 20; 

  if( GetCameraControlRegister( camera, 0x80C, &val ) != DC1394_SUCCESS ) 
    dgc_die("Could not get white balance\n");
  uint wbal_red  = (val >> 8) & 0x00000fff; 
  uint wbal_blue = val >> 20; 

  if( GetCameraControlRegister( camera, 0x820, &val ) != DC1394_SUCCESS ) 
    dgc_die("Could not get gain\n");
  uint gain = val >> 20; 

  if( GetCameraControlRegister( camera, 0x82C, &val ) != DC1394_SUCCESS ) 
    dgc_die("Could not get temperature\n");
  uint temperature = val >> 20; 

  dgc_info("shutter=%4d, bright=%4d, exposure=%4d, "
	   "wb=%4d,%4d, gain=%4d, temp=%4d\n",
	   shutter, brightness, exposure, wbal_red, wbal_blue, 
	   gain, temperature );
}

void 
ladybug_print_image_brightness_v2() 
{
  typedef union unionfloat_t {
    uint int_value;
    float value;
  } unionfloat;

  unionfloat shutter, brightness, exposure, gain, hue, saturation, gamma;

  if( GetCameraControlRegister( camera, 0x908, 
				&exposure.int_value   ) != DC1394_SUCCESS ) 
    dgc_die("Could not get exposure #2\n");
  if( GetCameraControlRegister( camera, 0x918, 
				&shutter.int_value    ) != DC1394_SUCCESS ) 
    dgc_die("Could not get shutter #2\n");
  if( GetCameraControlRegister( camera, 0x928, 
				&gain.int_value       ) != DC1394_SUCCESS ) 
    dgc_die("Could not get gain #2\n");
  if( GetCameraControlRegister( camera, 0x938,
				&brightness.int_value ) != DC1394_SUCCESS ) 
    dgc_die("Could not get brightness #2\n");
  if( GetCameraControlRegister( camera, 0x948,
				&gamma.int_value      ) != DC1394_SUCCESS ) 
    dgc_die("Could not get gamma #2\n");
  if( GetCameraControlRegister( camera, 0x978, 
				&hue.int_value        ) != DC1394_SUCCESS ) 
    dgc_die("Could not get hue #2\n");
  if( GetCameraControlRegister( camera, 0x988, 
				&saturation.int_value ) != DC1394_SUCCESS ) 
    dgc_die("Could not get saturation #2\n");

  dgc_info("shutter=%7.2f, bright=%7.2f, exposure=%7.2f, "
	   "gain=%7.2f, hue=%7.2f, sat=%7.2f, gamma=%7.2f\n",
	   shutter.value, brightness.value, exposure.value, 
	   gain.value, hue.value, saturation.value, gamma.value);
}

void 
ladybug_set_jpeg_compression( unsigned int value ) 
{
  uint jpeg_compr;
  unsigned int final_value = value - jpeg_compression_modifier;
  
  if( GetCameraControlRegister( camera, 0x1a20, 
				&jpeg_compr ) != DC1394_SUCCESS ) {
    dgc_die("Could not get JPEG compression factor #1\n");
  }
  jpeg_compr = 0x80000000 + final_value;
  if( SetCameraControlRegister( camera, 0x1a20, 
				jpeg_compr )  != DC1394_SUCCESS ) {
    dgc_die("Could not set JPEG compression factor\n");
  }
  if( GetCameraControlRegister( camera, 0x1a20, 
				&jpeg_compr ) != DC1394_SUCCESS ) {
    dgc_die("Could not get JPEG compression factor #2\n");
  }
  if( (jpeg_compr & 0xff) != final_value ) {
    dgc_die("Error setting JPEG compression factor: %d instead of %d\n", 
		jpeg_compr & 0xff, final_value );
  }
}

void 
ladybug_set_operation_mode() 
{
  int operation_mode = DC1394_OPERATION_MODE_1394B;
  fprintf(stderr, "Operation mode                    :     ");
  if( dc1394_video_set_operation_mode( camera, DC1394_OPERATION_MODE_1394B ) == DC1394_SUCCESS ) {
    fprintf( stderr, "1394b (800 Mbit)\n");
  } else {
    if(dc1394_video_set_operation_mode(camera, DC1394_OPERATION_MODE_LEGACY) != DC1394_SUCCESS )
      dgc_die("Error setting 1394 operation mode\n");
    operation_mode = DC1394_OPERATION_MODE_LEGACY;
    fprintf( stderr, "1394a (400 Mbit)\n");
  }
}

void 
ladybug_set_gamma(double gamma) 
{
  if (dc1394_feature_set_value(camera, DC1394_FEATURE_GAMMA, 
                               (int)(gamma*1000)) != DC1394_SUCCESS ) {
    dgc_die("Could not set gamma value\n");
  }
  if (dc1394_feature_set_power(camera, DC1394_FEATURE_GAMMA, 
			       DC1394_ON) != DC1394_SUCCESS ) {
    dgc_die("Could not enable gamma correction\n");
  }
}

void 
ladybug_set_whitebalance(int wb_blue, int wb_red) 
{
  if (dc1394_feature_whitebalance_set_value(camera, wb_blue, 
					    wb_red ) != DC1394_SUCCESS ) {
    dgc_die("Could not set white balance value\n");
  }
  if (dc1394_feature_set_power(camera, DC1394_FEATURE_WHITE_BALANCE, 
			       DC1394_ON) != DC1394_SUCCESS ) {
    dgc_die("Could not enable whitebalance\n");
  }
}

void 
ladybug_set_exposure(int exposure_value) 
{
  switch (ladybug_version) {
  case LADYBUG2:
    if (dc1394_feature_set_value(camera, DC1394_FEATURE_EXPOSURE, 
				 exposure_value) != DC1394_SUCCESS ) 
      dgc_die("Could not set gamma value\n");
    if (dc1394_feature_set_mode(camera, DC1394_FEATURE_EXPOSURE, 
				DC1394_FEATURE_MODE_MANUAL) != DC1394_SUCCESS ) 
      dgc_die("Could not enable gamma correction\n");
    break;
  case LADYBUG3:
    if (dc1394_feature_set_mode(camera, DC1394_FEATURE_EXPOSURE, 
				DC1394_FEATURE_MODE_AUTO) != DC1394_SUCCESS ) 
      dgc_die("Could not enable gamma correction\n");
    break;
  case UNKNOWN:
    break;
  }
}

#define BPP  8000

void 
ladybug_set_byte_per_packet() 
{
  uint bpp, wanted_bpp = 0;
  if( dc1394_format7_get_recommended_byte_per_packet(camera, 
						     DC1394_VIDEO_MODE_FORMAT7_7, 
                                                     &bpp ) != DC1394_SUCCESS )
    dgc_die("dc1394_format7_get_recommended_byte_per_packet() failed\n");
  fprintf(stderr, "recommended bpp                   :     %d\n", bpp);
  
  if (ladybug_version==LADYBUG2) { 
    if (dc1394_format7_set_byte_per_packet(camera, DC1394_VIDEO_MODE_FORMAT7_7, 
					   BPP) !=DC1394_SUCCESS ) {
      dgc_die("Could not change Format7 bytes per packet");
    }
  }
    
  if (dc1394_format7_get_byte_per_packet(camera, DC1394_VIDEO_MODE_FORMAT7_7, 
                                         &bpp) != DC1394_SUCCESS )
    dgc_die("Could not query Format7 bytes per packet");

  switch (ladybug_version) {
  case LADYBUG2:
    wanted_bpp = LB2_BPP;
    break;
  case LADYBUG3:
    wanted_bpp = LB3_BPP;
    break;
  case UNKNOWN:
    dgc_die( "Camera type \'%s\' not known\n", camera->model );    
  }
  if (bpp != wanted_bpp) 
    dgc_die("Format7 bytes per packet set to %d instead of %d\n",bpp,BPP);
}

void 
ladybug_set_iso_speed() 
{
  dc1394speed_t speed;
  if (dc1394_video_get_iso_speed(camera, &speed) != DC1394_SUCCESS)
    dgc_die("Can't get ISO speed\n");
  fprintf( stderr, "ISO speed                         :     %d\n", speed);
  if (dc1394_video_set_iso_speed(camera, DC1394_ISO_SPEED_800)!=DC1394_SUCCESS)
    dgc_die("Can't set ISO speed\n");
  if (dc1394_video_get_iso_speed(camera, &speed) != DC1394_SUCCESS)
    dgc_die("Can't get ISO speed\n");
  if( speed != DC1394_ISO_SPEED_800 )
    dgc_die("ISO speed is %d not %d\n", speed, DC1394_ISO_SPEED_800);
}  

void 
ladybug_set_initial_image_sizepos() 
{
  uint val1, val2, width = 0, height = 0;

  if( dc1394_format7_get_image_size( camera, DC1394_VIDEO_MODE_FORMAT7_7, 
				     &val1, &val2 ) != DC1394_SUCCESS )
    dgc_die("cannot get image position \n");
  dgc_info("cur initial image position (width=%d,height=%d)\n", width, height );
  return;
  switch (ladybug_version) {
  case LADYBUG2:
    width  = LB2_JPEG_WIDTH;
    height = LB2_COMPRESSED_IMAGE_HEIGHT;
    break;
  case LADYBUG3:
    width  = LB3_JPEG_WIDTH;
    height = LB3_COMPRESSED_IMAGE_HEIGHT;
    break;
  case UNKNOWN:
    dgc_die( "Camera type \'%s\' not known\n", camera->model );    
  }
  
  if( dc1394_format7_set_image_size( camera, DC1394_VIDEO_MODE_FORMAT7_7, 
				     width, height ) != DC1394_SUCCESS )
    dgc_die("cannot set image position \n");
  if( dc1394_format7_get_image_size( camera, DC1394_VIDEO_MODE_FORMAT7_7, 
				     &val1, &val2 ) != DC1394_SUCCESS )
    dgc_die("cannot get image position \n");
  if( val1 != width || val2 != height )
    dgc_die("error setting image position. Got %d,%d instead of %d,%d\n",
		val1, val2, width, height );
  if( dc1394_format7_set_image_position( camera, DC1394_VIDEO_MODE_FORMAT7_7,
					 0, 0 ) != DC1394_SUCCESS )
    dgc_die("cannot set image position \n");
  if( dc1394_format7_get_image_position( camera, DC1394_VIDEO_MODE_FORMAT7_7, 
					 &val1, &val2 ) != DC1394_SUCCESS )
    dgc_die("cannot get image position \n");
  if( val1 != 0 || val2 != 0 )
    dgc_die("error setting image position. Got %d,%d instead of 0,0\n",
		val1, val2 );
  dgc_info("set initial image position (width=%d,height=%d)\n", width, height );
}

struct timeval *
ladybug_capture_get_dma_filltime()
{
  return(dc1394_capture_get_dma_filltime( camera ));
}

void
ladybug_capture_dma( void )
{
  dc1394error_t err;
  int           retry;
  
  err = dc1394_capture_dma(&camera, 1, DC1394_VIDEO1394_WAIT);
  if( err != DC1394_SUCCESS) {
    fprintf( stderr, "# ERROR: frame capture returned %d.     \n", err);
    short capture_worked = 0;
    for( retry = 2; retry < 20; retry++ ) {
      if(dc1394_capture_dma(&camera,1,DC1394_VIDEO1394_WAIT)==DC1394_SUCCESS) {
        capture_worked = 1;
        break;
      } else {
        fprintf(stderr, "# ERROR: no luck on try #%d\n", retry );
      }
    }
    if( !capture_worked ) {
      dc1394_video_set_transmission( camera, DC1394_OFF );
      dgc_die("can't capture frames, giving up\n");
    }
  }
  dma_buffer_locked = 1;
}


unsigned char *
ladybug_capture_get_dma_buffer( void )
{
  return(dc1394_capture_get_dma_buffer( camera ));
}

int
ladybug_capture_get_frames_behind( void )
{
  return(dc1394_capture_get_frames_behind(camera));
}

int
ladybug_capture_get_height( void )
{
  return(dc1394_capture_get_height(camera));
}

int
ladybug_capture_get_width( void )
{
  return(dc1394_capture_get_width(camera));
}

int
ladybug_capture_get_bytes_per_frame( void )
{
  return(dc1394_capture_get_bytes_per_frame(camera));
}

void
ladybug_unlock_dma_if_necessary() 
{
  if( dma_buffer_locked )
    dc1394_capture_dma_done_with_buffer(camera);
  dma_buffer_locked = 0;
}

void 
ladybug_init_firewire(int camera_number) 
{
  dc1394camera_t **camera_list = NULL;
  unsigned int camera_count;
  
  if( dc1394_find_cameras( &camera_list, &camera_count ) != DC1394_SUCCESS ) 
    dgc_die("dc1394_find_cameras() failed. No camera found\n");
  
  if( camera_count == 0 )
    dgc_die("No firewire camera found\n");
  if( camera_number >= (int)camera_count )
    dgc_die("Cannot open camera %d, only %d found\n", 
		camera_number, camera_count );
  
  dgc_info("found %d firewire cameras, using camera %d\n", 
	   camera_count, camera_number);
  camera = camera_list[camera_number];

  if (!strncasecmp(camera->model, "Compressor", 10)) {
    ladybug_version = LADYBUG2;
    dgc_info("found LADYBUG 2\n" );
  } else if (!strncasecmp(camera->model, "Ladybug3", 8)) {
    ladybug_version = LADYBUG3;
    dgc_info("found LADYBUG 3\n" );
  } else {
    dgc_die("unknown ladybug model: %s\n", camera->model);
  }
 
  if( dc1394_capture_stop(camera) != DC1394_SUCCESS )
    dgc_warning("cannot stop capture (may not be a problem)\n");

  if ( dc1394_video_set_transmission( camera, DC1394_OFF ) != DC1394_SUCCESS ) 
    dgc_die("unable to stop camera iso transmission\n");

  fprintf( stderr, "\n" );
  if( dc1394_print_camera_info( camera ) != DC1394_SUCCESS )
    dgc_die("dc1394_print_camera_info() failed\n");

  if( dc1394_cleanup_iso_channels_and_bandwidth(camera) != DC1394_SUCCESS )
    dgc_die("cannot cleanup iso channels\n");
    
  if( dc1394_video_set_mode( camera, 
			     DC1394_VIDEO_MODE_FORMAT7_7 ) != DC1394_SUCCESS )
    dgc_die("cannot set video mode 7\n");

  // set parameters
  ladybug_set_jpeg_compression( jpeg_current_compression );
  ladybug_set_gamma(gamma_correction);
  switch (ladybug_version) {
  case LADYBUG2:
    ladybug_set_whitebalance(LB2_WB_BLUE, LB2_WB_RED);   
    break;
  case LADYBUG3:
    ladybug_set_whitebalance(LB3_WB_BLUE, LB3_WB_RED);   
    break;
  case UNKNOWN:
    break;
  }
  ladybug_set_exposure(exposure_value);
  ladybug_set_operation_mode();
  ladybug_set_iso_speed();
  ladybug_set_byte_per_packet();
  ladybug_set_initial_image_sizepos();

  fprintf(stderr,"serial number - base              :     %d\n", 
	  ladybug_get_base_serial_no() );
  fprintf(stderr,"serial number - head              :     %d\n", 
	  ladybug_get_head_serial_no());

  /* Start capture and set DMA buffer / frame drop policy */
  sleep(1);
  int dma_buffer_size = 110000, ci_height = 0;
  switch (ladybug_version) {
  case LADYBUG2:
    ci_height = LB2_COMPRESSED_IMAGE_HEIGHT;
    break;
  case LADYBUG3:
    ci_height = LB3_COMPRESSED_IMAGE_HEIGHT;
    break;
  case UNKNOWN:
    break;
  }
  
  while(dc1394_capture_setup_dma(camera, (int)dma_buffer_size / ci_height,
				 DC1394_RING_BUFFER_NEXT) != DC1394_SUCCESS) { 
    dgc_warning("cannot setup capture with DMA size %d\n", dma_buffer_size);
    dma_buffer_size -= 10000;
    if( dma_buffer_size < 30000 )
      dgc_die( "unable to setup capture\n" );
  }
  
  unsigned int bpp;
  if (dc1394_format7_get_byte_per_packet(camera, DC1394_VIDEO_MODE_FORMAT7_7, 
					 &bpp) !=DC1394_SUCCESS)
    dgc_die("Could not query Format7 bytes per packet");
  fprintf( stderr, "Format7 bytes per packet          :     %d\n", bpp );
  
  /*-----------------------------------------------------------------------
   *  have the camera start sending us data
   *-----------------------------------------------------------------------*/
  if ( dc1394_video_set_transmission( camera, DC1394_ON ) != DC1394_SUCCESS ) 
    dgc_die("unable to start camera iso transmission\n");
}

int
ladybug_frame_size()
{
  dc1394error_t err;
  uint64_t total_bytes = 0;
  
  err = dc1394_format7_get_total_bytes(camera, DC1394_VIDEO_MODE_FORMAT7_7,
				       &total_bytes);
  
  return( (int) total_bytes );
}

void
ladybug_close_camera()
{
  dc1394_capture_stop( camera );
  dc1394_video_set_transmission( camera, DC1394_OFF );
}

int
ladybug_retrieve_frame( dgc::LadybugPacket *pkt )
{
  unsigned char   *ptr;
  struct timeval  *filltime;
  unsigned int     len;

  ladybug_unlock_dma_if_necessary();
  ladybug_capture_dma();

  ptr      = ladybug_capture_get_dma_buffer();
  filltime = ladybug_capture_get_dma_filltime();
  len      = ladybug_check_frame( ptr );
  
  if (len > 0) {
    pkt->len = len;
    memcpy( pkt->data, ptr, pkt->len );
    pkt->timestamp = filltime->tv_sec + (filltime->tv_usec / 1000000.0);
    return 1;
  }
  return 0;
}

}
