#ifndef LADYBUG_DC1394_H
#define LADYBUG_DC1394_H

#include <ladybug_interface.h>

#define LB_JPEG_COMPRESSION_FACTOR  92
#define LB_GAMMA_CORRECTION         2.5

#define LB2_BPP                     8000
#define LB2_JPEG_WIDTH              512 
#define LB2_JPEG_HEIGHT             384
#define LB2_COMPRESSED_IMAGE_HEIGHT 3950
#define LB2_WB_BLUE                 75
#define LB2_WB_RED                  80

#define LB3_BPP                     9792
#define LB3_JPEG_WIDTH              808
#define LB3_JPEG_HEIGHT             616
#define LB3_COMPRESSED_IMAGE_HEIGHT 3950
#define LB3_WB_BLUE                 600
#define LB3_WB_RED                  600


extern int              auto_exposure_value;
extern int              shutter_value;
extern int              gain_value;
extern bool             auto_settings;
extern bool             sync_velodyne;

namespace dgc {

unsigned int      ladybug_get_base_serial_no();
unsigned int      ladybug_get_head_serial_no();
void              ladybug_print_image_brightness_v1();
void              ladybug_print_image_brightness_v2();
void              ladybug_set_jpeg_compression( unsigned int value );
void              ladybug_set_operation_mode();
void              ladybug_set_gamma(double gamma);
void              ladybug_set_whitebalance(int wb_blue, int wb_red); 
void              ladybug_set_exposure(); 
void              ladybug_set_trigger(); 
void              ladybug_set_byte_per_packet(); 
void              ladybug_set_iso_speed();
void              ladybug_set_initial_image_sizepos();
void              ladybug_init_firewire(int camera_number);
int               ladybug_frame_size();
int               ladybug_retrieve_frame( LadybugPacket *pkt );
void              ladybug_close_camera( void );

typedef enum {
  UNKNOWN, 
  LADYBUG2, 
  LADYBUG3
} ladybug_version_t;

extern ladybug_version_t ladybug_version;

}

#endif
