#define GL_GLEXT_PROTOTYPES

#include <roadrunner.h>
#include <dirent.h>
#include <gui3D.h>
#include <logio.h>

#include <ladybug_video.h>

using namespace dgc;

#define MAX_NAME_LENGTH  256

bool 
LadybugVideo::uncompressData(unsigned char *data) 
{
  LadybugError result = LADYBUG_OK;

  /******************************************************/
  result = compressorInfo.parse( data, 0, 0 );
  /******************************************************/
  return result == LADYBUG_OK;
}

LadybugVideo::LadybugVideo( char *conf_filename )
{ 
//  char filename[MAX_NAME_LENGTH];
  cgeffect_demosaic_bilinear = NULL;
  
  for( int j = 0; j < 4; j++ )
    fj[j] = dgc_fastjpeg_init_decompress();


  readConfig( conf_filename );

  remap_width   = config.camera_width;
  remap_height  = config.camera_height;

  /*
  sprintf( filename, "%s/cylindermap.dat", calib_dir );
  readCylinderMap( filename );
  */

  for( int cam = 0; cam < LADYBUG_NUM_CAMERAS ; cam++ ) {
    if( config.camera[cam].use_camera ) {

      distortedImage[cam].width     = remap_width;
      distortedImage[cam].height    = remap_height;
      distortedImage[cam].nchannels = 4;
      distortedImage[cam].pix = 
	(unsigned char *) malloc(distortedImage[cam].width*
				 distortedImage[cam].height*
				 distortedImage[cam].nchannels);

      undistortedImage[cam].width     = remap_width;
      undistortedImage[cam].height    = remap_height;
      undistortedImage[cam].nchannels = 4;
      undistortedImage[cam].pix = 
	(unsigned char *) malloc(undistortedImage[cam].width*
				 undistortedImage[cam].height*
				 undistortedImage[cam].nchannels);
        
      for (int bayer = 0; bayer < 4; bayer++ ) {
	bayerImage[cam][bayer].width     = remap_width  / 2;
	bayerImage[cam][bayer].height    = remap_height / 2;
	bayerImage[cam][bayer].nchannels = 1;
	bayerImage[cam][bayer].pix = 
	  (unsigned char *) malloc(bayerImage[cam][bayer].width*
				   bayerImage[cam][bayer].height*
				   bayerImage[cam][bayer].nchannels);
      }
    } else {
      distortedImage[cam].pix   = NULL;
      undistortedImage[cam].pix = NULL;
      for (int bayer = 0; bayer < 4; bayer++ ) {
        bayerImage[cam][bayer].pix = NULL;
      }
    }
  }
}

