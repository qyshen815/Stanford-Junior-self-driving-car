#define GL_GLEXT_PROTOTYPES

#include <roadrunner.h>
#include <ladybug_video.h>
#include <gui3D.h>

using namespace dgc;

#define MAX(a, b)  (((a) > (b)) ? (a) : (b))


// computes the next higher power of two of "value"
inline int 
npo2( int value ) {
  int result = 2;
  while( result < value ) {
    result *= 2;
  }
  return result;
}


/* Checks if extension extName is supported by OpenGL implementation */
  int 
dgc_gl_extension_exists(char *extName) 
{
  int extNameLen = strlen(extName);
  const GLubyte *ext_string = glGetString(GL_EXTENSIONS);
  char *p = (char *)ext_string;
  if( p == NULL ) {
    dgc_die("glGetString() returns NULL. OpenGL not initialized\n");
  }
  char *end = p + strlen(p);

  while (p < end) {
    int n = strcspn(p, " ");
    if ((extNameLen == n) && (strncmp(extName, p, n) == 0)) {
      return 1;
    }
    p += (n + 1);
  }
  return 0;
}

bool 
LadybugVideo::nvidiaAPIavailable() {
  static int use_nvidia_api = 0;

  if( use_nvidia_api==0 ) {
    use_nvidia_api = 1+dgc_gl_extension_exists("GL_EXT_pixel_buffer_object");
    if( use_nvidia_api==2 )
      dgc_info("GL_EXT_pixel_buffer_object supported\n");
    else
      dgc_info("GL_EXT_pixel_buffer_object NOT FOUND, using slower blocking calls\n");
  }

  return use_nvidia_api == 2;
}

  bool 
LadybugVideo::getImageDistorted( int iCamera )
{
  int i, j, jpeg_size[4];
  static int firsttime = TRUE;
  static unsigned char *buffer[6][4];
  static GLuint pbo_handle[LADYBUG_NUM_CAMERAS][4];
  const unsigned char* jpeg_data[4];
  int   width, height;

  if( !cameraInUse(iCamera) )
    return false;

  /*if( compressorInfo.images() != 24 ) {
    dgc_error("compressorInfo.images() = %d != 24 : %s", 
    compressorInfo.images(), strerror(errno));
    return false;
    }*/
  if( compressorInfo.totalSize() <= 0 )
    dgc_fatal_error("compressorInfo.totalSize() = %d <= 0", 
        compressorInfo.totalSize() );

  for( i=0; i<4; i++ ) {
    const LadybugCompressorHeaderInfo::ImageInfo* pimageinfo = 
      compressorInfo.getInfo( iCamera * 4 + i );
    jpeg_size[i] = pimageinfo->size;
    jpeg_data[i] = pimageinfo->pData;
  }

  if (firsttime) {
    glGenBuffersARB(24, (GLuint*)pbo_handle);
    for( i=0; i<LADYBUG_NUM_CAMERAS; i++ ) {
      for( j=0; j<4; j++ ) {
        if( cameraInUse(i) ) {
          width  = npo2(bayerImage[i][j].width);
          height = npo2(bayerImage[i][j].height);
          glGenTextures(1, (GLuint*)&(bayer_texture_id[i][j]) );
          glBindTexture(GL_TEXTURE_2D, bayer_texture_id[i][j]);
          glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
          buffer[i][j] = 
            (unsigned char *) malloc( width * height * sizeof(unsigned char) );
          glTexImage2D(GL_TEXTURE_2D, 0, 1, width, height,
              0, GL_LUMINANCE, GL_UNSIGNED_BYTE, buffer[i][j]);
          glBindTexture(GL_TEXTURE_2D, bayer_texture_id[i][j]);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
          glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, pbo_handle[i][j]);
          glBufferDataARB(GL_PIXEL_UNPACK_BUFFER_ARB, 
              bayerImage[i][j].width * bayerImage[i][j].height,
              NULL, GL_DYNAMIC_DRAW);
          glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, 0);
        }
      }
    }
    firsttime = FALSE;
  }

  unsigned char *original_buffer[4];
  if( nvidiaAPIavailable() ) {
    for( int iBayerChannel = 0; iBayerChannel < 4; iBayerChannel++ ) {
      original_buffer[iBayerChannel] = bayerImage[iCamera][iBayerChannel].pix;
      glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, 
          pbo_handle[iCamera][iBayerChannel]);
      unsigned char * buffer = 
        static_cast<unsigned char *>(glMapBufferARB(GL_PIXEL_UNPACK_BUFFER_EXT, 
              GL_WRITE_ONLY));
      bayerImage[iCamera][iBayerChannel].pix = buffer;
    }
  }

  for( int iBayerChannel=0; iBayerChannel<4; iBayerChannel++ ) {

    int iRBCorr;
    const LadybugCompressorHeaderInfo::ImageInfo* pimageinfo = 
      compressorInfo.getInfo( iCamera * 4 + iBayerChannel );

    switch(iBayerChannel) {
    case 0:
      iRBCorr = 3;
      break;
    case 1:
      iRBCorr = 1;
      break;
    case 2:
      iRBCorr = 2;
      break;
    case 3:
      iRBCorr = 0;
      break;

    }
    dgc_fastjpeg_decompress_memory( fj[iBayerChannel], 
        pimageinfo->pData, 
        pimageinfo->size,
        &(bayerImage[iCamera][iRBCorr]), 
        NULL );
  }

  if( nvidiaAPIavailable() ) {
    for( int iBayerChannel = 0; iBayerChannel < 4; iBayerChannel++ ) {
      glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, pbo_handle[iCamera][iBayerChannel]);
      if (!glUnmapBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB))
        dgc_die("Couldn't unmap pixel buffer. Exiting\n");
      glBindTexture(GL_TEXTURE_2D, bayer_texture_id[iCamera][iBayerChannel]);
      // copy buffer contents into the texture
      glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 
          bayerImage[iCamera][iBayerChannel].width,
          bayerImage[iCamera][iBayerChannel].height,
          GL_LUMINANCE, GL_UNSIGNED_BYTE,
          NULL);
      glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, 0);
      bayerImage[iCamera][iBayerChannel].pix = original_buffer[iBayerChannel];
    }
  } else {
    for( int iBayerChannel = 0; iBayerChannel < 4; iBayerChannel++ ) {
      glBindTexture(GL_TEXTURE_2D, bayer_texture_id[iCamera][iBayerChannel]);
      glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 
          bayerImage[iCamera][iBayerChannel].width,
          bayerImage[iCamera][iBayerChannel].height,
          GL_LUMINANCE, GL_UNSIGNED_BYTE,
          bayerImage[iCamera][iBayerChannel].pix);
    }
  }

  return true;
}

  double
compute_falloff_correction(double distance_from_center, int falloff)
{
  double gamma = 1.0;
  if (falloff) {
    double corr =  pow(dgc_square(1 + distance_from_center / 1000.0), 
        1 / gamma);
    return 0.45 * corr;
  } else {
    return 0.5;
  }
}

  void
LadybugVideo::drawGL(int camera, int falloff)
{
  int texture_size = 1024;
  glPushMatrix();

  glTranslatef(-40.0 * config.camera[camera].tx, 
      -40.0 * config.camera[camera].ty, 
      -40.0 * config.camera[camera].tz);

  glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_COMBINE_ARB);
  glTexEnvf(GL_TEXTURE_ENV, GL_COMBINE_RGB_ARB, GL_MODULATE);
  glTexEnvf(GL_TEXTURE_ENV, GL_SOURCE0_RGB_ARB, GL_TEXTURE);
  glTexEnvi(GL_TEXTURE_ENV, GL_RGB_SCALE_ARB, 2);

  // Draw image
  glBindTexture(GL_TEXTURE_2D, texture_id[camera]);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  double distance_fact = 40.0;

  dgc_ladybug_grid_p grid = &config.camera[camera].distorted_cylinder_grid; 

  glEnable(GL_TEXTURE_2D);
  glBegin(GL_QUADS);

  for(int y = 0; y < 768; y += 32) {
    for(int x = 0; x < 1024; x += 32) {
      int x2 = x / 32;
      int y2 = y / 32;
      int xn = x + 32 < 1024 ? x + 32 : x + 31;
      int yn = y + 32 <  768 ? y + 32 : y + 31;

      double radius = hypot(x - config.camera[camera].distorted_cx, 
          y - config.camera[camera].distorted_cy);
      double color = compute_falloff_correction(radius,falloff);
      distance_fact = 40.0 * sqrt(1 +  radius / 800.0);
      glColor4f(color, color, color,0.1);

      glTexCoord3f(((float)x + 0.5) / texture_size,    
          ((float)y + 0.5) / texture_size,
          0.2 );
      glVertex3f(distance_fact * grid->coefs[3 * (y2 * grid->width + x2) + 0],
          distance_fact * grid->coefs[3 * (y2 * grid->width + x2) + 1],
          distance_fact * grid->coefs[3 * (y2 * grid->width + x2) + 2]);

      radius = hypot(xn - config.camera[camera].distorted_cx, 
          y  - config.camera[camera].distorted_cy);
      color = compute_falloff_correction(radius,falloff);
      distance_fact = 40.0 * sqrt(1 +  radius / 800.0);
      glColor3f(color, color, color);

      glTexCoord2f(((float)xn + 0.5) / texture_size,    
          ((float)y  + 0.5) / texture_size);
      glVertex3f(distance_fact * grid->coefs[3 * (y2 * grid->width + x2 + 1) + 0],
          distance_fact * grid->coefs[3 * (y2 * grid->width + x2 + 1) + 1],
          distance_fact * grid->coefs[3 * (y2 * grid->width + x2 + 1) + 2]);

      radius = hypot(xn - config.camera[camera].distorted_cx, 
          yn - config.camera[camera].distorted_cy);
      color = compute_falloff_correction(radius,falloff);
      distance_fact = 40.0 * sqrt(1 +  radius/800.0);
      glColor3f(color, color, color);

      glTexCoord2f(((float)xn + 0.5) / texture_size,    
          ((float)yn + 0.5) / texture_size); 
      glVertex3f(distance_fact * grid->coefs[3 * ((y2 + 1) * grid->width + x2 + 1) + 0],
          distance_fact * grid->coefs[3 * ((y2 + 1) * grid->width + x2 + 1) + 1],
          distance_fact * grid->coefs[3 * ((y2 + 1) * grid->width + x2 + 1) + 2]);

      radius = hypot(x  - config.camera[camera].distorted_cx, 
          yn - config.camera[camera].distorted_cy );
      color = compute_falloff_correction(radius,falloff);
      distance_fact = 40.0 * sqrt(1 +  radius / 800.0);
      glColor3f(color, color, color);

      glTexCoord2f(((float)x  + 0.5) / texture_size,    
          ((float)yn + 0.5) / texture_size);
      glVertex3f(distance_fact * grid->coefs[3 * ((y2 + 1) * grid->width + x2) + 0],
          distance_fact * grid->coefs[3 * ((y2 + 1) * grid->width + x2) + 1],
          distance_fact * grid->coefs[3 * ((y2 + 1) * grid->width + x2) + 2]);
    }
  }
  glEnd();
  glDisable(GL_TEXTURE_2D);
  glTexEnvi(GL_TEXTURE_ENV, GL_RGB_SCALE_ARB, 1);
  glPopMatrix();
}

