#include <stdio.h>
#include <iostream>
#include <cmath>
#include <sstream>

#include <dc1394stereo.h>
#include <camera_interface.h>

namespace vlr {

DC1394Stereo::DC1394Stereo() : left_cam_(NULL), right_cam_(NULL), left_frame_(NULL), right_frame_(NULL),
    open_(false), frame_num_(0) {
  d_ = dc1394_new();
}

DC1394Stereo::~DC1394Stereo() {
  if (open_) {
    try {
      stopCapture();
    }
      catch(Exception& e) {
        throw Exception ("Failed to stop image capture: " + e.what());
      }
    try {
      close();
    }
    catch(Exception& e) {
      throw Exception ("Failed to close camera(s): " + e.what());
    }
  }

  dc1394_free(d_);
}

void DC1394Stereo::openSingleCam(uint64_t guid, dc1394color_coding_t color_coding, dc1394speed_t speed, dc1394camera_t*& cam, CameraParams& params) {

  if (!(cam = dc1394_camera_new(d_, guid))) {
    throw Exception("Could not initialize camera");
  }

  errCheck(dc1394_video_get_supported_modes(cam, &params.video_modes_), "Can't get video modes");

  // select highest res mode:
  int32_t i;
  for (i = params.video_modes_.num - 1; i >= 0; i--) {
    if (!dc1394_is_video_mode_scalable(params.video_modes_.modes[i])) {
      dc1394_get_color_coding_from_video_mode(cam, params.video_modes_.modes[i], &params.coding_);
      if (params.coding_ == color_coding) {
        params.video_mode_ = params.video_modes_.modes[i];
        break;
      }
    }
  }

  if (i < 0) {
    close();
    throw Exception("Could not get a valid video mode");
  }

  errCheck(dc1394_get_color_coding_from_video_mode(cam, params.video_mode_, &params.coding_), "Could not get color coding");

    // get highest framerate
  errCheck(dc1394_video_get_supported_framerates(cam, params.video_mode_, &params.framerates_), "Could not get framrates");

  params.framerate_ = params.framerates_.framerates[params.framerates_.num - 1];

  dc1394operation_mode_t mode;
  if(speed == DC1394_ISO_SPEED_800) {
    mode = DC1394_OPERATION_MODE_1394B;
  }
  else {
    mode = DC1394_OPERATION_MODE_LEGACY;
  }

  errCheck(dc1394_video_set_operation_mode(cam, mode), "Could not set operation mode");
  
  errCheck(dc1394_video_set_iso_speed(cam, speed), "Could not set iso speed");

  errCheck(dc1394_video_set_mode(cam, params.video_mode_), "Could not set video mode");

  errCheck(dc1394_video_set_framerate(cam, DC1394_FRAMERATE_7_5), "Could not set frame rate");
//  errCheck(dc1394_video_set_framerate(cam, params.framerate_), "Could not set frame rate");

  errCheck(dc1394_capture_setup(cam, 10, DC1394_CAPTURE_FLAGS_DEFAULT),
      "Could not setup camera - make sure that the video mode and frame rate are supported");

  // report camera's features
  errCheck(dc1394_feature_get_all(cam, &params.features_), "Could not get feature set");
  dc1394_feature_print_all(&params.features_, stdout);
}

void DC1394Stereo::listBus() {
dc1394camera_list_t* list = NULL;
errCheck(dc1394_camera_enumerate(d_, &list), "Could not enumerate cameras");

for (int32_t i = 0; i < (int32_t) list->num; i++) {
  std::cout << i << ". GUID: " << list->ids[i].guid << std::endl;
  }

dc1394_camera_free_list(list);
}

void DC1394Stereo::open(uint64_t left_guid, uint64_t right_guid, dc1394color_coding_t color_coding, dc1394speed_t speed) {

  if (open_) {
    try {
      close();
    }
    catch (Exception& e) {
      throw Exception("Failed to close camera(s) before re-opening: " + e.what());
    }
  }

  dc1394camera_list_t* list = NULL;
  errCheck(dc1394_camera_enumerate(d_, &list), "Could not enumerate cameras");


  if (list->num < 2) {
    throw Exception("Less than 2 cameras are connected..but would be required for stereo processing.");
  }

  int32_t left_idx = -1, right_idx = -1;
  for (int32_t i = 0; i < (int32_t) list->num; i++) {
    if (list->ids[i].guid == left_guid) {
      left_idx = i;
    }
    else if (list->ids[i].guid == right_guid) {
      right_idx = i;
    }
  }

  dc1394_camera_free_list(list);

  if (left_idx < 0) {
    std::stringstream s;
    s << "Could not find left camera (GUID: " << left_guid << ")";
    throw Exception(s.str());
  }
  if (right_idx < 0) {
    std::stringstream s;
    s << "Could not find left camera (GUID: " << right_guid << ")";
    throw Exception(s.str());
  }
    // set flag here to make sure things are closed correctly in case of an error
  open_ = true;

  try {
    openSingleCam(left_guid, color_coding, speed, left_cam_, left_params_);
  }
  catch (Exception& e) {
    std::stringstream s;
    s << "Could not open left camera (GUID: " << left_guid << "): " << e.what();
    throw Exception(s.str());
  }
  try {
    openSingleCam(right_guid, color_coding, speed, right_cam_, right_params_);
  }
  catch (Exception& e) {
    std::stringstream s;
    s << "Could not open right camera (GUID: " << right_guid << "): " << e.what();
    throw Exception(s.str());
  }
}

void DC1394Stereo::close() {

  if (left_cam_) {
    errCheck(dc1394_video_set_transmission(left_cam_, DC1394_OFF), "");
    errCheck(dc1394_capture_stop(left_cam_), "");
    dc1394_camera_free(left_cam_);
    left_cam_ = NULL;
  }
  if (right_cam_) {
    errCheck(dc1394_video_set_transmission(right_cam_, DC1394_OFF), "");
    errCheck(dc1394_capture_stop(right_cam_), "");
    dc1394_camera_free(right_cam_);
    right_cam_ = NULL;
  }

  open_ = false;
}

void DC1394Stereo::startCapture() {
  errCheck(dc1394_video_set_transmission(left_cam_, DC1394_ON), "Could not start iso transmission for left camera");
  errCheck(dc1394_video_set_transmission(right_cam_, DC1394_ON), "Could not start iso transmission for right camera");
}

void DC1394Stereo::stopCapture() {
  errCheck(dc1394_video_set_transmission(left_cam_, DC1394_OFF), "Could not stop iso transmission for left camera");
  errCheck(dc1394_video_set_transmission(right_cam_, DC1394_OFF), "Could not stop iso transmission for right camera");
}

void DC1394Stereo::setROISingleCam(dc1394camera_t* cam, CameraParams& params, int32_t x, int32_t y, int32_t width, int32_t height) {

if (params.video_mode_ < DC1394_VIDEO_MODE_FORMAT7_MIN) {
  throw Exception(__PRETTY_FUNCTION__ + std::string(": Video mode does not support ROI's"));
}

errCheck(dc1394_format7_set_image_position(cam, params.video_mode_, x, y), "Could not set ROI position");
errCheck(dc1394_format7_set_image_size(cam, params.video_mode_, width, height), "Could not set ROI size");

uint32_t bpp;
errCheck(dc1394_format7_get_recommended_packet_size(cam, params.video_mode_, &bpp), "Could not get recommended packet size");
errCheck(dc1394_format7_set_packet_size(cam, params.video_mode_, bpp), "Could not set packet size");
}

void DC1394Stereo::setROI(int32_t x, int32_t y, int32_t width, int32_t height) {
  try{
    setROISingleCam(left_cam_, left_params_, x, y, width, height);
  }
  catch(Exception& e) {
   throw Exception("Could not set ROI for left camera: " + e.what());
  }

  try{
    setROISingleCam(right_cam_, right_params_, x, y, width, height);
  }
  catch(Exception& e) {
   throw Exception("Could not set ROI for right camera: " + e.what());
  }
}

void DC1394Stereo::getImage(dgc::CameraImage& image) {
  // TODO: sync properly
  errCheck(dc1394_capture_dequeue(left_cam_, DC1394_CAPTURE_POLICY_WAIT, &left_frame_), "Could not capture frame from left camera");
  errCheck(dc1394_capture_dequeue(right_cam_, DC1394_CAPTURE_POLICY_WAIT, &right_frame_), "Could not capture frame from right camera");
  double dt = left_frame_->timestamp - right_frame_->timestamp;
    printf("time difference: %lf\n",dt);
  if(std::abs(dt) > 0.5/std::min(left_params_.framerate_, right_params_.framerate_)) {
    std::cout << "Warning: Stereo images probably out of sync, time difference: " << dt << "\n";
  }

  uint32_t width, height;
  dc1394_get_image_size_from_video_mode(left_cam_, left_params_.video_mode_, &width, &height);

  // TODO: make sure both cameras run in the same mode...or be more flexible here
  uint8_t depth;
  uint16_t format;
  uint8_t channels;

  try{
    depthAndFormatAndChannels(left_params_, depth, format, channels);
  }
  catch(Exception& e) {
    throw Exception("Could not determine image properties: " + e.what());
  }

  uint32_t size = width * height * channels * depth/8;
  if (image.info.width != width || image.info.height != height || image.info.channels != channels || !image.data) {
    if(image.data) {
      delete[] image.data;
    }
    image.data = new uint8_t[2*size];
    image.info.width = width;
    image.info.height = height;
    image.info.padded_width = width;
    image.num_bytes = size;
  }

  image.info.guid = 0;
  image.info.channels = 2*channels;
  image.info.depth = depth;
  image.info.format = format;
  image.info.camera_number = 0;
  image.frame_num = frame_num_;
  image.timestamp = left_frame_->timestamp / 1000000.0;

  memcpy((void*) image.data, left_frame_->image, size);
  memcpy((void*) &image.data[size], right_frame_->image, size);

  // return frames to ring buffers (unlock ring buffer entries)
  errCheck(dc1394_capture_enqueue(left_cam_, left_frame_) , "Could not return left camera frame to ring buffer");
  errCheck(dc1394_capture_enqueue(right_cam_, right_frame_) , "Could not return right camera frame to ring buffer");

  frame_num_++;
}

uint32_t DC1394Stereo::write(dgc::CameraInterface& interface) {
  dgc::CameraImage image;
  getImage(image);

  if (interface.WriteImage(&image) < 0) {
    throw Exception("Could not write image to camera interface.");
  }

  return image.num_bytes;
}

void DC1394Stereo::depthAndFormatAndChannels(CameraParams& params, uint8_t& depth, uint16_t& format, uint8_t& channels) {

  switch(params.coding_) {
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
}
