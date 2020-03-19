#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <serial.h>
#include <dc1394/control.h>
#include <dc1394/register.h>
#include <roadrunner.h>

#include "ladybug_dc1394.h"

unsigned int ladybug_check_frame( unsigned char *ptr );

static double    gamma_correction          = LB_GAMMA_CORRECTION;
int              auto_exposure_value;
int              shutter_value;
int              gain_value;
bool             auto_settings;
bool             sync_velodyne;

static int       jpeg_current_compression  = LB_JPEG_COMPRESSION_FACTOR;
static int       jpeg_compression_modifier = 0;


static dc1394camera_t *camera = NULL;

namespace dgc {

ladybug_version_t  ladybug_version = UNKNOWN;

unsigned int 
ladybug_get_base_serial_no() 
{
  static uint val = (uint)-1;
  int err;

  if( val == (uint)-1 ) {
    err = dc1394_get_control_register( camera, 0x1f20, &val );
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
    err = dc1394_get_control_register( camera, 0x1f80, &val );
    if( err != DC1394_SUCCESS ) 
      dgc_die("Could not read serial number, err=%d, val=%d\n", err, val);
  }
  return val;
}

void
ladybug_print_image_brightness_v1() 
{
  uint val;
  if( dc1394_get_control_register( camera, 0x81c, &val ) != DC1394_SUCCESS ) 
    dgc_die("Could not get shutter speed factor #1\n");
  uint shutter = val >> 20; 

  if( dc1394_get_control_register( camera, 0x800, &val ) != DC1394_SUCCESS ) 
    dgc_die("Could not get brightness\n");
  uint brightness = val >> 20; 

  if( dc1394_get_control_register( camera, 0x804, &val ) != DC1394_SUCCESS ) 
    dgc_die("Could not get exposure\n");
  uint exposure = val >> 20; 

  if( dc1394_get_control_register( camera, 0x80C, &val ) != DC1394_SUCCESS ) 
    dgc_die("Could not get white balance\n");
  uint wbal_red  = (val >> 8) & 0x00000fff; 
  uint wbal_blue = val >> 20; 

  if( dc1394_get_control_register( camera, 0x820, &val ) != DC1394_SUCCESS ) 
    dgc_die("Could not get gain\n");
  uint gain = val >> 20; 

  if( dc1394_get_control_register( camera, 0x82C, &val ) != DC1394_SUCCESS ) 
    dgc_die("Could not get temperature\n");
  uint temperature = val >> 20; 

  dgc_info("shutter=%4d, bright=%4d, exposure=%4d, "
	   "wb=%4d,%4d, gain=%4d, temp=%4d\n",
	   shutter, brightness, exposure, 
	   wbal_red, wbal_blue, gain, temperature );
}

void 
ladybug_print_image_brightness_v2() 
{
  typedef union unionfloat_t {
    uint int_value;
    float value;
  } unionfloat;

  unionfloat shutter, brightness, exposure, gain, hue, saturation, gamma;

  if( dc1394_get_control_register( camera, 0x908, 
				   &exposure.int_value ) != DC1394_SUCCESS ) 
    dgc_die("Could not get exposure #2\n");
  if( dc1394_get_control_register( camera, 0x918, 
				   &shutter.int_value ) != DC1394_SUCCESS ) 
    dgc_die("Could not get shutter #2\n");
  if( dc1394_get_control_register( camera, 0x928, 
				   &gain.int_value ) != DC1394_SUCCESS ) 
    dgc_die("Could not get gain #2\n");
  if( dc1394_get_control_register( camera, 0x938, 
				   &brightness.int_value ) != DC1394_SUCCESS ) 
    dgc_die("Could not get brightness #2\n");
  if( dc1394_get_control_register( camera, 0x948, 
				   &gamma.int_value ) != DC1394_SUCCESS ) 
    dgc_die("Could not get gamma #2\n");
  if( dc1394_get_control_register( camera, 0x978, 
				   &hue.int_value ) != DC1394_SUCCESS ) 
    dgc_die("Could not get hue #2\n");
  if( dc1394_get_control_register( camera, 0x988, 
				   &saturation.int_value ) != DC1394_SUCCESS ) 
    dgc_die("Could not get saturation #2\n");

  dgc_info("shutter=%7.2f, bright=%7.2f, exposure=%7.2f, gain=%7.2f, "
	   "hue=%7.2f, sat=%7.2f, gamma=%7.2f\n",
	   shutter.value, brightness.value, exposure.value, 
	   gain.value, hue.value, saturation.value, gamma.value);
}

void 
ladybug_set_jpeg_compression( unsigned int value ) 
{
  uint jpeg_compr;
  unsigned int final_value = value - jpeg_compression_modifier;
  
  if( dc1394_get_control_register( camera, 0x1a20, 
				   &jpeg_compr ) != DC1394_SUCCESS ) 
    dgc_die("Could not get JPEG compression factor #1\n");
  jpeg_compr = 0x80000000 + final_value;
  if( dc1394_set_control_register( camera, 0x1a20, 
				   jpeg_compr )  != DC1394_SUCCESS ) 
    dgc_die("Could not set JPEG compression factor\n");
  if( dc1394_get_control_register( camera, 0x1a20,
				   &jpeg_compr ) != DC1394_SUCCESS ) 
    dgc_die("Could not get JPEG compression factor #2\n");
  if( (jpeg_compr & 0xff) != final_value )
    dgc_die("Error setting JPEG compression factor: %d instead of %d\n", 
            jpeg_compr & 0xff, final_value );
}

void 
ladybug_set_operation_mode() 
{
  int operation_mode = DC1394_OPERATION_MODE_1394B;
  fprintf(stderr, "Operation mode                    :     ");
  if( dc1394_video_set_operation_mode( camera, DC1394_OPERATION_MODE_1394B ) == DC1394_SUCCESS ) {
    fprintf( stderr, "1394b (800 Mbit)\n");
  } else {
    if( dc1394_video_set_operation_mode( camera, DC1394_OPERATION_MODE_LEGACY ) != DC1394_SUCCESS )
      dgc_die("Error setting 1394 operation mode\n");
    operation_mode = DC1394_OPERATION_MODE_LEGACY;
    fprintf( stderr, "1394a (400 Mbit)\n");
  }
}

void 
ladybug_set_gamma(double gamma) 
{
  // Set Gamma Correction
  //  if (dc1394_feature_on_off(control_handle, camera.node, 6, TRUE)!=DC1394_SUCCESS) {
  if (dc1394_feature_set_value(camera, DC1394_FEATURE_GAMMA, 
                               (int)(gamma*1000)) != DC1394_SUCCESS ) 
    dgc_die("Could not set gamma value\n");
  if (dc1394_feature_set_power(camera, DC1394_FEATURE_GAMMA, DC1394_ON) != DC1394_SUCCESS ) 
    dgc_die("Could not enable gamma correction\n");
}

void 
ladybug_set_whitebalance(int wb_blue, int wb_red) 
{
  // Set Gamma Correction
  //  if (dc1394_feature_on_off(control_handle, camera.node, 6, TRUE)!=DC1394_SUCCESS) {
  if (dc1394_feature_whitebalance_set_value(camera, wb_blue, wb_red ) != DC1394_SUCCESS ) 
    dgc_die("Could not set white balance value\n");
  if (dc1394_feature_set_power(camera, DC1394_FEATURE_WHITE_BALANCE, DC1394_ON) != DC1394_SUCCESS ) 
    dgc_die("Could not enable whitebalance\n");
}

void 
ladybug_set_exposure() 
{
  switch(auto_settings) {
  case true:
    dgc_info("Auto exposure mode");
    if (dc1394_feature_set_mode(camera, DC1394_FEATURE_EXPOSURE, 
        DC1394_FEATURE_MODE_AUTO) != DC1394_SUCCESS ) 
      dgc_die("Could not set auto exposure\n");
    if (dc1394_feature_set_mode(camera, DC1394_FEATURE_SHUTTER, 
        DC1394_FEATURE_MODE_AUTO) != DC1394_SUCCESS ) 
      dgc_die("Could not set auto exposure\n");
    if (dc1394_feature_set_mode(camera, DC1394_FEATURE_GAIN, 
        DC1394_FEATURE_MODE_AUTO) != DC1394_SUCCESS ) 
      dgc_die("Could not set auto exposure\n");
    break;
  case false:
    dgc_info("Manual exposure mode");
    dgc_info(" --- auto_exposure_value: %d", auto_exposure_value);
    dgc_info(" --- gain_value: %d", gain_value);
    dgc_info(" --- shutter_value: %d", shutter_value);
    if (dc1394_feature_set_power(camera, DC1394_FEATURE_EXPOSURE, 
        DC1394_ON) != DC1394_SUCCESS ) 
      dgc_die("Could not set exposure control to on\n");
    if (dc1394_feature_set_mode(camera, DC1394_FEATURE_EXPOSURE, 
        DC1394_FEATURE_MODE_MANUAL) != DC1394_SUCCESS ) 
      dgc_die("Could not set manual exposure\n");
    if (dc1394_feature_set_value(camera, DC1394_FEATURE_EXPOSURE, 
         auto_exposure_value) != DC1394_SUCCESS ) 
      dgc_die("Could not set exposure value\n");
    if (dc1394_feature_set_mode(camera, DC1394_FEATURE_SHUTTER,
        DC1394_FEATURE_MODE_MANUAL) != DC1394_SUCCESS )
      dgc_die("Could not enable manual shutter\n");
    if (dc1394_feature_set_value(camera, DC1394_FEATURE_SHUTTER, 
         shutter_value) != DC1394_SUCCESS ) 
      dgc_die("Could not set shutter value\n");
    if (dc1394_feature_set_mode(camera, DC1394_FEATURE_GAIN,
        DC1394_FEATURE_MODE_MANUAL) != DC1394_SUCCESS )
      dgc_die("Could not enable manual gain\n");
    if (dc1394_feature_set_value(camera, DC1394_FEATURE_GAIN, 
         gain_value) != DC1394_SUCCESS ) 
      dgc_die("Could not set gain value\n");
    break;
  }
}

void 
ladybug_set_byte_per_packet() 
{
  uint bpp, wanted_bpp = 0;
  if( dc1394_format7_get_recommended_packet_size(camera, 
						 DC1394_VIDEO_MODE_FORMAT7_7, 
						 &bpp ) != DC1394_SUCCESS )
    dgc_die("dc1394_format7_get_recommended_byte_per_packet() failed\n");
  fprintf(stderr, "recommended bpp                   :     %d\n", bpp);

  if (ladybug_version==LADYBUG2) {
    if ( dc1394_format7_set_packet_size(camera, 
					DC1394_VIDEO_MODE_FORMAT7_7, 
					LB2_BPP) !=DC1394_SUCCESS )
      dgc_die("Could not change Format7 bytes per packet\n");
  }

  if (dc1394_format7_get_packet_size(camera, 
				     DC1394_VIDEO_MODE_FORMAT7_7, 
				     &bpp) != DC1394_SUCCESS )
      dgc_die("Could not query Format7 bytes per packet\n");

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
    dgc_die( "Format7 bytes per packet set to %d instead of %d\n", 
		 bpp, wanted_bpp );
}

void 
ladybug_set_iso_speed() 
{
  dc1394speed_t speed;
  if (dc1394_video_get_iso_speed(camera, &speed) != DC1394_SUCCESS)
    dgc_die("Can't get ISO speed\n");
  if (dc1394_video_set_iso_speed(camera, DC1394_ISO_SPEED_800) != DC1394_SUCCESS)
    dgc_die("Can't set ISO speed\n");
  if (dc1394_video_get_iso_speed(camera, &speed) != DC1394_SUCCESS)
    dgc_die("Can't get ISO speed\n");
  if( speed != DC1394_ISO_SPEED_800 )
    dgc_die("ISO speed is %d not %d\n", speed, DC1394_ISO_SPEED_800);
  fprintf( stderr, "ISO speed                         :     " );
  switch (speed) {
  case DC1394_ISO_SPEED_100:
    fprintf( stderr, "100\n" );
    break;
  case DC1394_ISO_SPEED_200:
    fprintf( stderr, "200\n" );
    break;
  case DC1394_ISO_SPEED_400:
    fprintf( stderr, "400\n" );
    break;
  case DC1394_ISO_SPEED_800:
    fprintf( stderr, "800\n" );
    break;
  case DC1394_ISO_SPEED_1600:
    fprintf( stderr, "1600\n" );
    break;
  case DC1394_ISO_SPEED_3200:
    fprintf( stderr, "3200\n" );
    break;
  }
}  


void
ladybug_print_video_mode( dc1394video_mode_t mode )
{   
  switch (mode) {
  case DC1394_VIDEO_MODE_160x120_YUV444:
    fprintf( stderr, "DC1394_VIDEO_MODE_160x120_YUV444" );
    break;
  case DC1394_VIDEO_MODE_320x240_YUV422:
    fprintf( stderr, "DC1394_VIDEO_MODE_320x240_YUV422" );
    break;
  case DC1394_VIDEO_MODE_640x480_YUV411:
    fprintf( stderr, "DC1394_VIDEO_MODE_640x480_YUV411" );
    break;
  case DC1394_VIDEO_MODE_640x480_YUV422:
    fprintf( stderr, "DC1394_VIDEO_MODE_640x480_YUV422" );
    break;
  case DC1394_VIDEO_MODE_640x480_RGB8:
    fprintf( stderr, "DC1394_VIDEO_MODE_640x480_RGB8" );
    break;
  case DC1394_VIDEO_MODE_640x480_MONO8:
    fprintf( stderr, "DC1394_VIDEO_MODE_640x480_MONO8" );
    break;
  case DC1394_VIDEO_MODE_640x480_MONO16:
    fprintf( stderr, "DC1394_VIDEO_MODE_640x480_MONO16" );
    break;
  case DC1394_VIDEO_MODE_800x600_YUV422:
    fprintf( stderr, "DC1394_VIDEO_MODE_800x600_YUV422" );
    break;
  case DC1394_VIDEO_MODE_800x600_RGB8:
    fprintf( stderr, "DC1394_VIDEO_MODE_800x600_RGB8" );
    break;
  case DC1394_VIDEO_MODE_800x600_MONO8:
    fprintf( stderr, "DC1394_VIDEO_MODE_800x600_MONO8" );
    break;
  case DC1394_VIDEO_MODE_1024x768_YUV422:
    fprintf( stderr, "DC1394_VIDEO_MODE_1024x768_YUV422" );
    break;
  case DC1394_VIDEO_MODE_1024x768_RGB8:
    fprintf( stderr, "DC1394_VIDEO_MODE_1024x768_RGB8" );
    break;
  case DC1394_VIDEO_MODE_1024x768_MONO8:
    fprintf( stderr, "DC1394_VIDEO_MODE_1024x768_MONO8" );
    break;
  case DC1394_VIDEO_MODE_800x600_MONO16:
    fprintf( stderr, "DC1394_VIDEO_MODE_800x600_MONO16" );
    break;
  case DC1394_VIDEO_MODE_1024x768_MONO16:
    fprintf( stderr, "DC1394_VIDEO_MODE_1024x768_MONO16" );
    break;
  case DC1394_VIDEO_MODE_1280x960_YUV422:
    fprintf( stderr, "DC1394_VIDEO_MODE_1280x960_YUV422" );
    break;
  case DC1394_VIDEO_MODE_1280x960_RGB8:
    fprintf( stderr, "DC1394_VIDEO_MODE_1280x960_RGB8" );
    break;
  case DC1394_VIDEO_MODE_1280x960_MONO8:
    fprintf( stderr, "DC1394_VIDEO_MODE_1280x960_MONO8" );
    break;
  case DC1394_VIDEO_MODE_1600x1200_YUV422:
    fprintf( stderr, "DC1394_VIDEO_MODE_1600x1200_YUV422" );
    break;
  case DC1394_VIDEO_MODE_1600x1200_RGB8:
    fprintf( stderr, "DC1394_VIDEO_MODE_1600x1200_RGB8" );
    break;
  case DC1394_VIDEO_MODE_1600x1200_MONO8:
    fprintf( stderr, "DC1394_VIDEO_MODE_1600x1200_MONO8" );
    break;
  case DC1394_VIDEO_MODE_1280x960_MONO16:
    fprintf( stderr, "DC1394_VIDEO_MODE_1280x960_MONO16" );
    break;
  case DC1394_VIDEO_MODE_1600x1200_MONO16:
    fprintf( stderr, "DC1394_VIDEO_MODE_1600x1200_MONO16" );
    break;
  case DC1394_VIDEO_MODE_EXIF:
    fprintf( stderr, "DC1394_VIDEO_MODE_EXIF" );
    break;
  case DC1394_VIDEO_MODE_FORMAT7_0:
    fprintf( stderr, "DC1394_VIDEO_MODE_FORMAT7_0" );
    break;
  case DC1394_VIDEO_MODE_FORMAT7_1:
    fprintf( stderr, "DC1394_VIDEO_MODE_FORMAT7_1" );
    break;
  case DC1394_VIDEO_MODE_FORMAT7_2:
    fprintf( stderr, "DC1394_VIDEO_MODE_FORMAT7_2" );
    break;
  case DC1394_VIDEO_MODE_FORMAT7_3:
    fprintf( stderr, "DC1394_VIDEO_MODE_FORMAT7_3" );
    break;
  case DC1394_VIDEO_MODE_FORMAT7_4:
    fprintf( stderr, "DC1394_VIDEO_MODE_FORMAT7_4" );
    break;
  case DC1394_VIDEO_MODE_FORMAT7_5:
    fprintf( stderr, "DC1394_VIDEO_MODE_FORMAT7_5" );
    break;
  case DC1394_VIDEO_MODE_FORMAT7_6:
    fprintf( stderr, "DC1394_VIDEO_MODE_FORMAT7_6" );
    break;
  case DC1394_VIDEO_MODE_FORMAT7_7:
    fprintf( stderr, "DC1394_VIDEO_MODE_FORMAT7_7" );
    break;
  default:
    fprintf( stderr, "unknown mode %d", mode );
  }
}

void
ladybug_print_video_modes()
{
  unsigned int i;
  dc1394video_modes_t 	video_modes;
  if ( dc1394_video_get_supported_modes( camera, 
					 &video_modes ) != DC1394_SUCCESS ) {
    dgc_die("can't get supported modes\n");
  }
  fprintf( stderr,"Numper of supported video modes   :     %d\n", 
	   video_modes.num );
  for (i=0; i<video_modes.num; i++) {
    fprintf( stderr,"     [" );
    ladybug_print_video_mode( video_modes.modes[i] );
    fprintf( stderr,"]\n" );
  }
  dc1394video_mode_t video_mode;
  dc1394_video_get_mode( camera, &video_mode );
  fprintf( stderr,"Current video mode                :     " );
  ladybug_print_video_mode( video_mode );
  fprintf( stderr,"\n" );
}


void
ladybug_get_framerate()
{
  // get highest framerate
  dc1394framerates_t 	framerates;
  dc1394framerate_t 	framerate;
  if (dc1394_video_get_supported_framerates( camera, 
					     DC1394_VIDEO_MODE_FORMAT7_7, 
					     &framerates ) != DC1394_SUCCESS ){
    dgc_error("can't get a framerate\n");
  }
  framerate = framerates.framerates[ framerates.num-1 ];
  fprintf( stderr, "Frame rate                        : %d\n", framerate );
}


void 
ladybug_set_initial_image_sizepos() 
{
  uint val1, val2, width = 0, height = 0;

  if( dc1394_format7_get_image_size( camera, DC1394_VIDEO_MODE_FORMAT7_7, 
				     &val1, &val2 ) != DC1394_SUCCESS )
    dgc_die("cannot get image position \n");
  dgc_info("cur initial image position (width=%d,height=%d)\n",
	   width, height );
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
  dgc_info("set initial image position (width=%d,height=%d)\n",
	   width, height );
}

void ladybug_set_trigger()
{
  switch(sync_velodyne) {
  case true:
    if( dc1394_software_trigger_set_power( camera, DC1394_OFF )
        != DC1394_SUCCESS) {
      dgc_die("Could not disable software trigger\n");
    }
    if( dc1394_external_trigger_set_power( camera, DC1394_ON )
        != DC1394_SUCCESS) {
    dgc_die("Could not enable external trigger\n");
    }
    break;
  case false:
    if( dc1394_software_trigger_set_power( camera, DC1394_ON )
        != DC1394_SUCCESS) {
      dgc_die("Could not enable software trigger\n");
    }
    if( dc1394_external_trigger_set_power( camera, DC1394_OFF )
        != DC1394_SUCCESS) {
    dgc_die("Could not disable external trigger\n");
    }
    break;
  }
  if( dc1394_external_trigger_set_mode( camera, DC1394_TRIGGER_MODE_0 )
      != DC1394_SUCCESS) {
    dgc_die("Could not set external trigger mode\n");
  }

  if( dc1394_external_trigger_set_polarity( camera, DC1394_TRIGGER_ACTIVE_HIGH )
      != DC1394_SUCCESS) {
    dgc_die("Could not set external trigger polarity\n");
  }


} 

dc1394video_frame_t *
ladybug_capture( void )
{
  dc1394video_frame_t *frame=NULL;
  dc1394error_t err;
  
  err = dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_WAIT, &frame);
  if( err != DC1394_SUCCESS) {
    return(NULL);
  }
  return(frame);
}

void
ladybug_capture_enqueue( dc1394video_frame_t *frame )
{
  dc1394_capture_enqueue(camera, frame);
}

void
ladybug_close_camera( void )
{
  dc1394_capture_stop( camera );
  dc1394_video_set_transmission( camera, DC1394_OFF );
  dc1394_camera_free( camera );
}

void
ladybug_init_firewire(int camera_number) 
{
  dc1394_t * d;
  dc1394camera_list_t * list;
  dc1394error_t err;

  dgc_info("using dc1394 v2 library ...\n");
  
  d = dc1394_new ();

  dgc_info("looking for DC1394 camera ...\n");
  err=dc1394_camera_enumerate (d, &list);
  if (err!=DC1394_SUCCESS)
    dgc_die("failed to enumerate cameras\n");
  
  if (list->num == 0) {
    dgc_die("no firewire camera found\n");
  }
  
  camera = dc1394_camera_new (d, list->ids[camera_number].guid);
  if (!camera) {
    dgc_error("failed to initialize camera with guid %llx", 
	      (long long unsigned int)list->ids[camera_number].guid);
  }
  dc1394_camera_free_list (list);
  dgc_info("using camera with GUID %llx\n", 
      (long long unsigned int)camera->guid);
  
  dgc_info("resetting camera\n" );
  dc1394_camera_reset(camera);

  // WTF!?!?!?!? - mvs 9/10/2010
  if (0) {
    if ( dc1394_camera_set_power( camera, DC1394_ON )!= DC1394_SUCCESS ) {
      dgc_die("unable to turn on camera\n");
    }
    
    if ( dc1394_external_trigger_set_power( camera, 
					    DC1394_OFF )!= DC1394_SUCCESS ) {
      dgc_die("unable to turn on camera\n");
    }
    dc1394featureset_t features;
    if ( dc1394_feature_get_all( camera, &features )!= DC1394_SUCCESS ) {
      dgc_die("unable get features\n");
    } else {
      dc1394_feature_print_all( &features, stderr ); 
    }
  }

  ladybug_set_trigger();

  if( dc1394_capture_stop(camera) != DC1394_SUCCESS ) {
    dgc_warning("cannot stop capture (may not be a problem)\n");
  }

  dc1394switch_t tmission;
  if (dc1394_video_get_transmission( camera, &tmission )!=DC1394_SUCCESS ) {
    dgc_die("unable to get transmission status\n");
  }
  if (tmission == DC1394_ON) {
    dgc_info("camera transmission is ON\n" );
    if ( dc1394_video_set_transmission( camera, DC1394_OFF )!= DC1394_SUCCESS ) 
      dgc_die("unable to stop camera iso transmission\n");
    if (dc1394_video_get_transmission( camera, &tmission ) != DC1394_SUCCESS ) {
      dgc_die("unable to get transmission status\n");
    } else if (tmission == DC1394_ON) {
      dgc_die("unable to turn off transmission\n");
    }
  } else {
    dgc_info("camera transmission is OFF\n" );
  }

  if (!strncasecmp(camera->model, "Compressor", 10)) {
    ladybug_version = LADYBUG2;
    dgc_info("found LADYBUG 2\n" );
  } else if (!strncasecmp(camera->model, "Ladybug3", 8)) {
    ladybug_version = LADYBUG3;
    dgc_info("found LADYBUG 3\n" );
  } else {
    dgc_die("unknown ladybug model: %s\n", camera->model);
  }

  if( dc1394_video_set_mode( camera, DC1394_VIDEO_MODE_FORMAT7_7 ) != 
      DC1394_SUCCESS ) {
    dgc_die("cannot set video mode 7\n");
  }
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
  
  ladybug_set_exposure();
  ladybug_set_initial_image_sizepos();

  if( dc1394_camera_print_info( camera, stdout ) != DC1394_SUCCESS )
    dgc_die("dc1394_print_camera_info() failed\n");

  switch (ladybug_version) {
  case LADYBUG3:
    unsigned int bpp;
    if (dc1394_format7_get_packet_size(camera, DC1394_VIDEO_MODE_FORMAT7_7, 
				       &bpp) !=DC1394_SUCCESS)
      dgc_die("Could not query Format7 bytes per packet");
    fprintf( stderr, "\nFormat7 bytes per packet          :     %d\n", bpp );
  default:
    break;
  }

  ladybug_print_video_modes();
  ladybug_set_operation_mode();
  ladybug_set_iso_speed();
  ladybug_set_byte_per_packet();

  fprintf(stderr,"serial number - base              :     %d\n", 
	  ladybug_get_base_serial_no() );
  fprintf(stderr,"serial number - head              :     %d\n", 
	  ladybug_get_head_serial_no());

  ladybug_print_image_brightness_v1();
  ladybug_print_image_brightness_v2(); 
  switch (ladybug_version) {
  case LADYBUG2:
    err = dc1394_format7_set_roi( camera,
				  DC1394_VIDEO_MODE_FORMAT7_7,
				  DC1394_COLOR_CODING_MONO8,
				  LB2_BPP, 
				  0, 0,
				  LB2_JPEG_WIDTH, LB2_COMPRESSED_IMAGE_HEIGHT );
    if (err != DC1394_SUCCESS) {
      dgc_die("Could not set roi for LB2");
    }
    break;
  case LADYBUG3:
    err = dc1394_format7_set_roi( camera,
				  DC1394_VIDEO_MODE_FORMAT7_7,
				  DC1394_COLOR_CODING_MONO8,
				  LB3_BPP, 
				  0, 0,
				  LB3_JPEG_WIDTH, LB3_COMPRESSED_IMAGE_HEIGHT );
    if (err != DC1394_SUCCESS) {
      dgc_die("Could not set roi for LB3\n");
    }
    break;
  case UNKNOWN:
    break;
  }

  err=dc1394_capture_setup(camera, 10, DC1394_CAPTURE_FLAGS_DEFAULT);
  if (err != DC1394_SUCCESS) {
    dc1394_reset_bus(camera);
    dgc_die("Could setup capture, reset the bus\n");
  }

  /*-----------------------------------------------------------------------
   *  have the camera start sending us data
   *-----------------------------------------------------------------------*/
  dgc_info("start transmission ...\n" );
  if ( dc1394_video_set_transmission( camera, DC1394_ON ) != DC1394_SUCCESS ) 
    dgc_die("unable to start camera iso transmission\n");

  if (dc1394_video_get_transmission( camera, &tmission )!=DC1394_SUCCESS ) {
    dgc_die("unable to get transmission status\n");
  }
  if (tmission == DC1394_ON) {
    dgc_info("camera transmission is ON\n" );
  } else {
    dgc_info("camera transmission is OFF\n" );
  }
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

int
ladybug_retrieve_frame( LadybugPacket *pkt )
{
  unsigned int     len = 0;
  dc1394video_frame_t *frame;

  frame = ladybug_capture();
  if (frame) {
    len = ladybug_check_frame( frame->image );
    if (len > 0) {
      pkt->len = len;
      memcpy( pkt->data, frame->image, pkt->len );
      pkt->timestamp = frame->timestamp / 1000000.0;
    }
    ladybug_capture_enqueue(frame);
    return len;
  }
  return 0;
}

}
