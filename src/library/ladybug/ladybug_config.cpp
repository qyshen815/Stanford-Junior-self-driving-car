#include <roadrunner.h>
#include <logio.h>
#include <ladybug_video.h>

using namespace dgc;

#define MY_WARP_SHIFT          5
#define MY_WARP_SHIFT2         10
#define MY_WARP_MASK           ((1 << MY_WARP_SHIFT) - 1)
#define MY_WARP_SIZE           (1<<MY_WARP_SHIFT)

inline bool 
beginsWith( char **s, char *start ) {
  bool result = strncmp( *s, start, strlen(start) ) == 0;
  if( result ) {
    *s += strlen(start);
    return true;
  } else {
    return false;
  }
}

#define MAX_LINE_LENGTH  100000

void 
LadybugVideo::readCameraConfig( FILE *f )
{
  dgc_ladybug_camera_config_t       camera;
  int                               w, h, n, i, cam = 0;
  char                              buf[MAX_LINE_LENGTH]; 
  static bool                       first_run[LADYBUG_NUM_CAMERAS] = { 1,1,1,1,1,1 };

  memset( &camera, 0, sizeof(dgc_ladybug_camera_config_t) );

  camera.distorted_cylinder_grid.coefs  = NULL;

  while( fgets( buf, MAX_LINE_LENGTH, f ) != NULL ) {
    char *pos;
    for( pos = buf; *pos==' ' || *pos=='\t'; pos++ ) {};

    if( beginsWith( &pos, "Id ") ) {
      cam = READ_INT( &pos );
      if( cam < 0 || cam > 5 ) {
	fprintf( stderr, "\n" );
        dgc_warning("configuration file has entry for cam %d\n", cam );
      }
      fprintf(stderr," %d", cam );
    }
    if( beginsWith( &pos, "focalLength ") ) 
      camera.f = 1024 * READ_DOUBLE(&pos);

    if( beginsWith( &pos, "CamToLadybugEulerZYX ") ) {
      camera.rx = READ_DOUBLE(&pos);
      camera.ry = READ_DOUBLE(&pos);
      camera.rz = READ_DOUBLE(&pos);
      camera.tx = READ_DOUBLE(&pos);
      camera.ty = READ_DOUBLE(&pos);
      camera.tz = READ_DOUBLE(&pos);
    }

    if( beginsWith( &pos, "Center ") ) {
      for( int i = 0; i < 3; i++ )
        camera.center[i] = READ_DOUBLE(&pos);
      camera.cx = READ_DOUBLE(&pos) * 1024;    /* + 3*/
      camera.cy = READ_DOUBLE(&pos) * 768;   
    }

    if( beginsWith( &pos, "TopLeft ") ) 
      for( int i = 0; i < 3; i++ )
        camera.topleft[i] = READ_DOUBLE(&pos);

    if( beginsWith( &pos, "TopRight ") ) 
      for( int i = 0; i < 3; i++ )
        camera.topright[i] = READ_DOUBLE(&pos);

    if( beginsWith( &pos, "BottomLeft ") ) 
      for( int i = 0; i < 3; i++ )
        camera.bottomleft[i] = READ_DOUBLE(&pos);

    if( beginsWith( &pos, "BottomRight ") ) 
      for( int i = 0; i < 3; i++ )
        camera.bottomright[i] = READ_DOUBLE(&pos);

    if( beginsWith( &pos, "RectifiedSpline ") ) 
      for( int i = 0; i < 2; i++ )
        camera.rectified_grid_id[i] = READ_INT(&pos);

    if( beginsWith( &pos, "DistortedSpline ") ) 
      for( int i = 0; i < 2; i++ )
        camera.distorted_grid_id[i] = READ_INT(&pos);

    if( beginsWith( &pos, "3DSurfaceSpline ") ) 
      for( int i = 0; i < 3; i++ )
        camera.surface3d_grid_id[i] = READ_INT(&pos);

    if( beginsWith( &pos, "falloffCorrection ") ) 
      camera.falloff_grid_id = READ_INT(&pos);

    if( beginsWith( &pos, "DistortedCenter ") ) {
      camera.distorted_cx = READ_DOUBLE(&pos);
      camera.distorted_cy = READ_DOUBLE(&pos);
    }

    if( beginsWith( &pos, "UseCamera ") ) {
      camera.use_camera = READ_INT(&pos);
    }

    if( beginsWith( &pos, "ProjTable ") ) {
      camera.distorted_cylinder_grid.width = READ_INT(&pos);
      camera.distorted_cylinder_grid.height = READ_INT(&pos);
      camera.distorted_cylinder_grid.coefs = 
	(float *) malloc( camera.distorted_cylinder_grid.width *
			  camera.distorted_cylinder_grid.height *
			  3 * sizeof(float) );
      dgc_test_alloc( camera.distorted_cylinder_grid.coefs );
      for (h=0; h<camera.distorted_cylinder_grid.height; h++) {
	for (w=0; w<camera.distorted_cylinder_grid.width; w++) {
	  for (i=0; i<3; i++) {
	    n = ((h*camera.distorted_cylinder_grid.width)+w)*3+i;
	    camera.distorted_cylinder_grid.coefs[n] = READ_FLOAT(&pos);
	  }
	}
      }
    }

    if( beginsWith( &pos, "EndCamera") ) {
      if( !first_run[cam] ) {
        free(config.camera[cam].falloff_grid.coefs);
        free(config.camera[cam].distorted_cylinder_grid.coefs);
        for( int i = 0; i < 2; i++ ) {
          free(config.camera[cam].rectified_grid[i].coefs);
          free(config.camera[cam].distorted_grid[i].coefs);
        }
        for( int i = 0; i < 3; i++ ) 
          free(config.camera[cam].surface3d_grid[i].coefs);
      }
      first_run[cam]=false;
      memcpy( &(config.camera[cam]), &camera, 
	      sizeof(dgc_ladybug_camera_config_t));

      if (camera.distorted_cylinder_grid.coefs==NULL) {
	fprintf( stderr, "\n# ERROR: no ProjTable is defined for camera %d\n", cam ); 
	exit(0);
      }

      config.camera[cam].distorted_cylinder_grid.coefs = 
	(float *) malloc( camera.distorted_cylinder_grid.width *
			  camera.distorted_cylinder_grid.height *
			  3 * sizeof(float) );
      dgc_test_alloc( config.camera[cam].distorted_cylinder_grid.coefs );
      memcpy( config.camera[cam].distorted_cylinder_grid.coefs, 
	      camera.distorted_cylinder_grid.coefs,
	      camera.distorted_cylinder_grid.width *
	      camera.distorted_cylinder_grid.height *
	      3 * sizeof(float) );
      free( camera.distorted_cylinder_grid.coefs );
      
      double scale_factor = (double) 1536 / 1024.0;
      config.camera[cam].f  *= scale_factor;
      config.camera[cam].cx *= scale_factor;
      config.camera[cam].cy *= scale_factor;

      return;
    }
  }

}

void 
LadybugVideo::readConfig( char * filename )
{
  char    buf[1000]; 
  FILE   *f;

  struct stat  statbuf;

  f = fopen( filename, "r");
  if( f==NULL ) {
    dgc_die( "cannot open config file %s\n", filename );
  } else {
    fstat( fileno(f), &statbuf );
    if( !S_ISREG( statbuf.st_mode ) ) {
      fclose(f);
      dgc_die( "cannot find config file %s\n", filename );
    }
    dgc_info("reading config file %s ... ", filename);
  }


  while( fgets( buf, 1000, f ) != NULL ) {
    char *pos;
    for( pos = buf; *pos==' ' || *pos=='\t'; pos++ ) {};
    if( beginsWith( &pos, "BeginCamera" ) ) {
      readCameraConfig(f);
    }
    if( beginsWith( &pos, "Resolution ") ) {
      config.camera_width = READ_INT(&pos);
      config.camera_height = READ_INT(&pos);
    }
    if( beginsWith( &pos, "Version ") ) {
      config.version = READ_INT(&pos);
    }

  }
  fprintf(stderr, " done\n");  
  fclose(f);

}

