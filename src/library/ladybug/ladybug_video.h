#ifndef LADYBUG_VIDEO_H
#define LADYBUG_VIDEO_H

#include <fastjpeg.h>
#include <Cg/cg.h>  
#include <Cg/cgGL.h>

#include <ladybug_interface.h>
#include <ladybug_frameparser.h>

namespace dgc {

#define MAX_LADYBUG_FRAME_SIZE  (4*1024*1024)
#define LADYBUG_DIST_FACTOR     1.0
#define LADYBUG_NUM_CAMERAS     6

#define _MAX_NAME_LENGTH        256

typedef struct {

  int                  width;
  int                  height;
  float               *coefs;

} dgc_ladybug_grid_t, *dgc_ladybug_grid_p;


typedef struct {

  int                  use_camera;

  double               f;
  double               cx;
  double               cy;
  double               rx, ry, rz;
  double               tx, ty, tz;

  double               center[3];
  double               topleft[3];
  double               topright[3];
  double               bottomleft[3];
  double               bottomright[3];
  
  int                  rectified_grid_id[2];
  int                  distorted_grid_id[2];
  int                  surface3d_grid_id[3];
  int                  falloff_grid_id;

  double               distorted_cx;
  double               distorted_cy;

  dgc_ladybug_grid_t   rectified_grid[2];
  dgc_ladybug_grid_t   distorted_grid[2];
  dgc_ladybug_grid_t   surface3d_grid[3];
  dgc_ladybug_grid_t   falloff_grid;
  dgc_ladybug_grid_t   distorted_cylinder_grid;

} dgc_ladybug_camera_config_t, *dgc_ladybug_camera_config_p;


typedef struct {

  dgc_ladybug_camera_config_t    camera[LADYBUG_NUM_CAMERAS];
  int                  camera_width;
  int                  camera_height;
  int                  version;

} dgc_ladybug_config_t, *dgc_ladybug_config_p;






class LadybugVideo {

 protected:
  
  int                            bayer_texture_id[LADYBUG_NUM_CAMERAS][4];
  char                          *ladybug_calib_directory;

  CGeffect                       cgeffect_demosaic_bilinear;
  CGcontext                      myCgContext;
  CGprofile                      myCgFragmentProfile;

  jpeg_decompress_struct        *fj[4];

  unsigned char                  pData[MAX_LADYBUG_FRAME_SIZE];

  int                            remap_width;
  int                            remap_height;
  dgc_ladybug_config_t           config;

 public:
  
  // raw images before debayering: B,G,G,R
  vlr::dgc_image_t                    bayerImage[LADYBUG_NUM_CAMERAS][4];
  vlr::dgc_image_t                    distortedImage[LADYBUG_NUM_CAMERAS];   
  vlr::dgc_image_t                    undistortedImage[LADYBUG_NUM_CAMERAS]; 

  LadybugCompressorHeaderInfo    compressorInfo;
  int                            texture_id[LADYBUG_NUM_CAMERAS];

  LadybugVideo(char *calib_directory );

  void           initializeCg();

  bool           uncompressData(unsigned char *data);

  bool           distortCoordinates( int camera, double w, double h,
				     double *resultw, double *resulth );

  bool           getImageDistorted( int camera );

  void           demosaicBilinear( int camera );

  bool           cameraInUse( int camera ) { return config.camera[camera].use_camera; }
  

  void           drawGL(int camera, int falloff);


private:

  void           readConfig( char * filename );
  void           readCameraConfig( FILE *fp );

  void           setupRenderToFbo( int width, int height, int format,
				   GLuint* texture, GLuint *framebuf,
				   GLuint *depthbuf );
  
  void           initTexture( GLuint* texture_handle, int width, int height, 
			      int num_channels, int format, int channel_order,
			      int internal_format = 4 );

  bool           nvidiaAPIavailable();
  void           checkForCgError(const char *situation);
  void           checkCgEffect( CGeffect effect );
  void           demosaicDrawGeometry(float width_ratio, float height_ratio, 
				      float ox=-0.5, float oy=-0.5, float ox2=0, float oy2=0);

};

} // namespace dgc

#endif

