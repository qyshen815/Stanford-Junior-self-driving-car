#ifndef DC1394STEREO_H_
#define DC1394STEREO_H_

#include <dc1394/dc1394.h>

#include <vlrException.h>
#include <global.h>
#include <camera_interface.h>

namespace vlr {

class DC1394Stereo {
public:
    class CameraParams {
    public:
      dc1394featureset_t features_;
      dc1394framerates_t framerates_;
      dc1394video_modes_t video_modes_;
      dc1394framerate_t framerate_;
      dc1394video_mode_t video_mode_;
      dc1394color_coding_t coding_;
    };

    DC1394Stereo();
    virtual ~DC1394Stereo();
    void open(uint64_t guid_left, uint64_t guid_right,
	      dc1394color_coding_t c_coding=DC1394_COLOR_CODING_RGB8,
	      dc1394speed_t speed=DC1394_ISO_SPEED_800);
    void close();
    void startCapture();
    void stopCapture();
    void setROI(int32_t h, int32_t v, int32_t width, int32_t height);
    void getImage(dgc::CameraImage& image);
    uint32_t write(dgc::CameraInterface& interface);
    void listBus();
    
private:
    inline void errCheck(dc1394error_t e, std::string base_err) {
      if(e != DC1394_SUCCESS) {
        close();
        throw Exception(__PRETTY_FUNCTION__ + std::string(": ") + base_err + std::string(": ") + dc1394_error_get_string(e));
      }
    }

    void openSingleCam(uint64_t guid, dc1394color_coding_t color_coding, dc1394speed_t speed, dc1394camera_t*& cam, CameraParams& params);
    void setROISingleCam(dc1394camera_t* cam, CameraParams& params, int32_t x, int32_t y, int32_t width, int32_t height);
    void depthAndFormatAndChannels(CameraParams& params, uint8_t& depth, uint16_t& format, uint8_t& channels);

private:
    dc1394camera_t* left_cam_, *right_cam_;
    CameraParams left_params_, right_params_;
    uint32_t width_, height_;
    dc1394video_frame_t* left_frame_, *right_frame_;
    dc1394_t* d_;
    bool open_;
    uint64_t frame_num_;
};

}
#endif
