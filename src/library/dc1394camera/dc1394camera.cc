#include <stdio.h>
#include <iostream>

#include <vlrException.h>
#include <global.h>
#include <dc1394camera.h>
#include <camera_interface.h>

using namespace dgc;

namespace vlr {

DC1394Camera::DC1394Camera() 
{
  d = dc1394_new ();
  camera = NULL;
  frame = NULL;
  _image = new CameraImage;
  _image->info.guid = 0;
  _image->info.height = 0;
  _image->info.width = 0;
  _image->frame_num = 0;
  _image->info.camera_number = -1;
}

DC1394Camera::~DC1394Camera()
{
  if(_image->info.guid >0) {
    stopCapture();
    if(!close()) {
      dgc_error( "Failed to close camera %lld\n", _image->info.guid );
      return;
    }
  }
  dc1394_free(d);
}

bool DC1394Camera::open(unsigned long long guid, 
			dc1394color_coding_t color_coding,
			dc1394speed_t speed ) 
{
  int i, nr;
  dc1394camera_list_t* list=NULL;
  
  if(_image->info.guid >0) {
    if(!close()) {
      return false;
    }
  }
  
  if(dc1394_camera_enumerate (d, &list) != DC1394_SUCCESS) {
    dgc_error( "Failed to enumerate cameras\n" );
    return false;
  }
  
  if (list->num == 0) {
    dgc_error( "No cameras found\n" );
    return false;
  }

  if (guid==0) {
    nr = 0;
  } else {
    nr = -1;
    for (i=0; i<(int)list->num; i++) {
      if (list->ids[i].guid==guid) {
	nr = i;
      }
    }
    if (nr<0) {
      dgc_error( "Could not find camera %llu\n", guid );
      return false;
    }
  }
  
  if(!(camera = dc1394_camera_new (d, list->ids[nr].guid))) {
    dgc_error( "Failed to initialize camera with guid %llu\n",
	       (unsigned long long int)list->ids[nr].guid );
    return false;
  }

  _image->info.guid = list->ids[nr].guid;
  _image->info.camera_number = nr;

  dgc_info( "Using camera with guid %llu\n", (unsigned long long int)list->ids[nr].guid );
  
  dc1394_camera_free_list(list);
  
  
  if(dc1394_video_get_supported_modes(camera, 
				      &video_modes) != DC1394_SUCCESS) {
    dgc_error( "Can't get video modes" );
    close();
    return false;
  }
  
  // select highest res mode:
  for (i=video_modes.num-1; i>=0; i--) {
    if (!dc1394_is_video_mode_scalable(video_modes.modes[i])) {
      dc1394_get_color_coding_from_video_mode(camera, 
					      video_modes.modes[i], 
					      &coding);
      if (coding==color_coding) {
	video_mode=video_modes.modes[i];
	break;
      }
    }
  }
  
  if (i < 0) {
    dgc_error( "Could not get a valid video mode\n" );
    close();
    return false;
  }
  
  if(dc1394_get_color_coding_from_video_mode(camera, video_mode, 
					     &coding) != DC1394_SUCCESS) {
    dgc_error( "Could not get color coding\n" );
    close();
    return false;
  }
  
  // get highest framerate
  if(dc1394_video_get_supported_framerates(camera, video_mode, 
					   &framerates) != DC1394_SUCCESS) {
    dgc_error( "Could not get framrates\n" );
    close();
    return false;
  }
  
  framerate=framerates.framerates[framerates.num-1];
 
  dc1394_video_set_operation_mode(camera, DC1394_OPERATION_MODE_1394B);
  dgc_info( "camera: %llx\n", (unsigned long long int) camera);
  if(dc1394_video_set_iso_speed(camera, speed ) != DC1394_SUCCESS) {
    dgc_error( "Could not set iso speed\n" );
    close();
    return false;
  }
  
  if(dc1394_video_set_mode(camera, video_mode) != DC1394_SUCCESS) {
    dgc_error( "Could not set video mode\n" );
    close();
    return false;
  }

  if(dc1394_video_set_framerate(camera, framerate) != DC1394_SUCCESS) {
    dgc_error( "Could not set framrate\n" );
    close();
    return false;
  }

  if(dc1394_capture_setup(camera, 10, 
			  DC1394_CAPTURE_FLAGS_DEFAULT) != DC1394_SUCCESS) {
    dgc_error( "Could not setup camera-\n"
	       "make sure that the video mode and framerate are\n"
	       "supported by your camera\n" );
    close();
    return false;
  }
  
    // disable all automatic value changes (AGC, AWB, ...)
    // and set them to values suited for traffic light detection

  allManual();

  // report camera's features
  if(dc1394_feature_get_all(camera,&features) !=DC1394_SUCCESS)	{
    std::cout << "Could not get feature set\n";
  } else {
    dc1394_feature_print_all(&features, stdout);
  }
  
  return true;
}

bool DC1394Camera::close()
{
  if(!camera) {
    return true;
  }
  
  dc1394_video_set_transmission(camera, DC1394_OFF);
  dc1394_capture_stop(camera);
  dc1394_camera_free(camera);
  
  camera = NULL;
  _image->info.guid = 0;

  return true;
}

bool DC1394Camera::startCapture()
{
  if(dc1394_video_set_transmission(camera, 
				   DC1394_ON) != DC1394_SUCCESS) {
    dgc_error( "Could not start camera iso transmission\n" );
    close();
    return false;
  }
  return true;
}
  
bool DC1394Camera::stopCapture()
{
  if(dc1394_video_set_transmission(camera, 
				   DC1394_OFF) != DC1394_SUCCESS) {
    dgc_error( "Could not stop camera iso transmission\n" );
    close();
    return false;
  }
  return true;
}

bool DC1394Camera::setROI(int h, int v, int width, int height) {

  unsigned int bpp;

  if(video_mode < DC1394_VIDEO_MODE_FORMAT7_MIN)
    return false;

  if(dc1394_format7_set_image_position(camera,video_mode,h,v) != 
      DC1394_SUCCESS) {
    dgc_error( "Could not set ROI position\n" );
    return false;
  }

  if(dc1394_format7_set_image_size(camera,video_mode,width,height) !=
      DC1394_SUCCESS) {
    dgc_error( "Could not set ROI size\n" );
    return false;
  }

  if(dc1394_format7_get_recommended_packet_size(camera,video_mode,&bpp) !=
      DC1394_SUCCESS ) {
    dgc_error( "Could not get recommended packet size\n" );
    return false;
  }

  if(dc1394_format7_set_packet_size(camera,video_mode,bpp) !=
      DC1394_SUCCESS ) {
    dgc_error( "Could not set packet size\n" );
    return false;
  }

  return true;
}

bool DC1394Camera::getImage(CameraImage* image)
{
  // capture one frame
  if(dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_WAIT, 
			    &frame) != DC1394_SUCCESS) {
    dgc_error( "Could not capture a frame\n" );
    close();
    return false;
  }
  
  dc1394_get_image_size_from_video_mode(camera, video_mode, 
					&width, &height);
  int size = width*height*sizeof(unsigned char);
  
  if (image==NULL) {
    image = (CameraImage*) malloc( sizeof(CameraImage) );
  } 
  if (image->info.width != width || image->info.height != height) {
    image->data = (unsigned char *) realloc( image->data, size );
    image->info.width=width;
    image->info.height=height;
    image->info.padded_width=width;
    image->num_bytes = size;
  }
  
  image->info.guid          = _image->info.guid;
  image->info.channels      = _image->info.channels;
  image->info.depth         = _image->info.depth;
  image->info.format        = _image->info.format;
  image->info.camera_number = _image->info.camera_number;
  image->frame_num          = _image->frame_num;
  image->timestamp          = frame->timestamp / 1000000.0;
  
  memcpy((void*)image->data, frame->image, size);
  
  // return frame to ring buffer (unlock ring buffer entry)
  if(dc1394_capture_enqueue(camera, frame) != DC1394_SUCCESS) {
    dgc_error( "Could not return entry to ring buffer\n" );
    close();
    return false;
  }
  
  _image->frame_num++;
  
  return true;
}

int DC1394Camera::writeImage(CameraInterface* interface)
{
  // capture one frame
  if(dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_WAIT, 
			    &frame) != DC1394_SUCCESS) {
    dgc_error( "Could not capture a frame\n" );
    close();
    return 0;
  }
  
  dc1394_get_image_size_from_video_mode(camera, video_mode, &width, &height);

  uint8_t depth; uint16_t format; uint8_t channels;
  depthAndFormatAndChannels(coding, depth, format, channels);
  int size = width*height*channels*depth/8;
  
  if (_image->info.width != width || 
      _image->info.height != height ||
      _image->info.channels != channels ||
      _image->info.depth != depth) {
    _image->info.width=width;
    _image->info.height=height;
    _image->info.padded_width=width;
    _image->info.channels = channels;
    _image->info.format = format;
    _image->info.depth = depth;
    _image->num_bytes = size;
  }
  
  _image->timestamp = frame->timestamp / 1000000.0;
  _image->data      = frame->image;

  if (interface->WriteImage(_image) < 0) {
    dgc_error( "Could not write image to camera interface.\n" );
    return 0;
  }
  
  // return frame to ring buffer (unlock ring buffer entry)
  if(dc1394_capture_enqueue(camera, frame) != DC1394_SUCCESS) {
    dgc_error( "Could not return entry to ring buffer\n" );
    close();
    return 0;
  }
  
  _image->frame_num++;
  
  return size;
}

void DC1394Camera::depthAndFormatAndChannels(const dc1394color_coding_t coding, uint8_t& depth, uint16_t& format, uint8_t& channels) {

  switch(coding) {
  case DC1394_COLOR_CODING_MONO8:
    depth=8;
    format = dgc::DGC_GRAY8_FMT;
    channels = 1;
    break;

  case DC1394_COLOR_CODING_YUV411:
    throw Exception(__PRETTY_FUNCTION__ + std::string(": Coding scheme DC1394_COLOR_CODING_YUV422 not supported (yet)"));
    break;

  case DC1394_COLOR_CODING_YUV422:
    throw Exception(__PRETTY_FUNCTION__ + std::string(": Coding scheme DC1394_COLOR_CODING_YUV422 not supported (yet)"));
    break;

  case DC1394_COLOR_CODING_YUV444:
    throw Exception(__PRETTY_FUNCTION__ + std::string(": Coding scheme DC1394_COLOR_CODING_YUV422 not supported (yet)"));
    break;

  case DC1394_COLOR_CODING_RGB8:
    depth=8;
    format = dgc::DGC_RGB8_FMT;
    channels = 3;
    break;

  case DC1394_COLOR_CODING_MONO16:
    depth=16;
    format = dgc::DGC_GRAY16L_FMT;  // Could also be DGC_GRAY16B_FMT, depends on camera internals...
    channels = 1;
    break;

  case DC1394_COLOR_CODING_RGB16:
    throw Exception(__PRETTY_FUNCTION__ + std::string(": Coding scheme DC1394_COLOR_CODING_RGB16 not supported (yet)"));
    break;

  case DC1394_COLOR_CODING_MONO16S:
    throw Exception(__PRETTY_FUNCTION__ + std::string(": Coding scheme DC1394_COLOR_CODING_MONO16S not supported (yet)"));
    break;

  case DC1394_COLOR_CODING_RGB16S:
    throw Exception(__PRETTY_FUNCTION__ + std::string(": Coding scheme DC1394_COLOR_CODING_RGB16S not supported (yet)"));
    break;

  case DC1394_COLOR_CODING_RAW8:
    throw Exception(__PRETTY_FUNCTION__ + std::string(": Coding scheme DC1394_COLOR_CODING_RAW8 not supported (yet)"));
    break;

  case DC1394_COLOR_CODING_RAW16:
    throw Exception(__PRETTY_FUNCTION__ + std::string(": Coding scheme DC1394_COLOR_CODING_RAW16 not supported (yet)"));
    break;

  default:
    throw Exception(__PRETTY_FUNCTION__ + std::string(": Unknown coding scheme"));
 }
}

void DC1394Camera::allManual() {

  for(int32_t i = DC1394_FEATURE_MIN; i <= DC1394_FEATURE_MAX; ++i) {
    if(dc1394_feature_set_mode(camera, (dc1394feature_t)i, DC1394_FEATURE_MODE_MANUAL) != DC1394_SUCCESS) {
      printf( "Could not enable manual mode for feature %i...but that's probably ok\n", i);
    }
  }

  if(dc1394_feature_whitebalance_set_value(camera, 640, 576) != DC1394_SUCCESS) {
    dgc_error( "Could not adjust white balance\n" );
  }

  if(dc1394_feature_set_value(camera, DC1394_FEATURE_GAIN, 60) != DC1394_SUCCESS) {
    dgc_error( "Could not adjust gain\n" );
  }

  if(dc1394_feature_set_value(camera, DC1394_FEATURE_EXPOSURE, 1) != DC1394_SUCCESS) {
    dgc_error( "Could not adjust exposure\n" );
  }

  if(dc1394_feature_set_value(camera, DC1394_FEATURE_SHARPNESS, 1532) != DC1394_SUCCESS) {
    dgc_error( "Could not adjust sharpness\n" );
  }

  if(dc1394_feature_set_value(camera, DC1394_FEATURE_SATURATION, 1278) != DC1394_SUCCESS) {
    dgc_error( "Could not adjust saturation\n" );
  }

  if(dc1394_feature_set_value(camera, DC1394_FEATURE_SHUTTER, 60) != DC1394_SUCCESS) {
    dgc_error( "Could not adjust shutter\n" );
  }
}

}
