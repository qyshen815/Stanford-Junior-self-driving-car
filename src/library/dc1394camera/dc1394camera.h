#ifndef _DC1394CAMERA_H
#define _DC1394CAMERA_H

#include <dc1394/dc1394.h>

#include <global.h>
#include <camera_interface.h>

namespace vlr {

class DC1394Camera {
public:
    DC1394Camera();
    virtual ~DC1394Camera();
    bool open(unsigned long long guid=0, 
	      dc1394color_coding_t c_coding=DC1394_COLOR_CODING_RGB8,
	      dc1394speed_t speed=DC1394_ISO_SPEED_800);
    bool close();
    bool startCapture();
    bool stopCapture();
    bool setROI(int h, int v, int width, int height);
    bool getImage(dgc::CameraImage *image);
    int  writeImage(dgc::CameraInterface* interface);
   
private:
void depthAndFormatAndChannels(const dc1394color_coding_t coding, uint8_t& depth, uint16_t& format, uint8_t& channels); 
void allManual();

private:
    dgc::CameraImage *_image;

    dc1394camera_t* camera;
    dc1394featureset_t features;
    dc1394framerates_t framerates;
    dc1394video_modes_t video_modes;
    dc1394framerate_t framerate;
    dc1394video_mode_t video_mode;
    dc1394color_coding_t coding;
    unsigned int width, height;
    dc1394video_frame_t* frame;
    dc1394_t* d;
    dc1394error_t err;
};

}
#endif
