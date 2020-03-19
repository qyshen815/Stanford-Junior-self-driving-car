#define GL_GLEXT_PROTOTYPES

#include <roadrunner.h>
#include <ladybug_video.h>
#include <gui3D.h>
#include "demosaic_bilinear.cgfx.h"
#include <ladybug_interface.h>

using namespace dgc;
using namespace vlr;

// computes the next higher power of two of "value"
inline int 
npo2( int value ) {
  int result = 2;
  while( result < value ) {
    result *= 2;
  }
  return result;
}

void 
LadybugVideo::checkForCgError(const char *situation) 
{
  CGerror error;
  const char *string = cgGetLastErrorString(&error);

  if (error != CG_NO_ERROR) {
    dgc_warning("(Cg) %s: %s\n",  situation, string);
    if (error == CG_COMPILER_ERROR) {
      dgc_warning("(Cg) %s\n", cgGetLastListing(myCgContext));
    }
    exit(1);
  }
}

void 
LadybugVideo::checkCgEffect( CGeffect effect ) 
{
  if (!effect) {
    const char *listing = cgGetLastListing(this->myCgContext);
    dgc_die( "checkCgEffect: Unable load effect '%s'\n", listing );
  }
  CGtechnique technique = cgGetFirstTechnique(effect);

  while (technique) {
    if (cgValidateTechnique(technique) == CG_FALSE)
      dgc_die("technique %s did not validate.\n", 
	      cgGetTechniqueName(technique));
    technique = cgGetNextTechnique(technique);
  }
}

void 
LadybugVideo::initTexture( GLuint* texture_handle, int width, int height, 
			   int num_channels, int format, int channel_order, 
			   int internal_format  ) 
{  
  // Initialize Textures
  if( *texture_handle == (GLuint)-1 || *texture_handle == 0 ) {
    glGenTextures(1, (GLuint*)texture_handle);
    dgc_info("init texture %d\n", *texture_handle);
    glBindTexture(GL_TEXTURE_2D, *texture_handle);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    int bytes_per_pixel = 1;
    switch( format ) {
    case GL_UNSIGNED_BYTE:
    case GL_BYTE:
      bytes_per_pixel = 1;
      break;  
    case GL_UNSIGNED_SHORT:
    case GL_SHORT:
    case GL_HALF_FLOAT_ARB:
      bytes_per_pixel = 2;
      break;  
    case GL_FLOAT:
      bytes_per_pixel = 4;
      break;  
    default:
      dgc_die("(hendrik) unsupported GL format %d\n", format);
    }
    int buffer_size = npo2(width) * npo2(height) * num_channels * bytes_per_pixel;
    static void* buffer = NULL;
    static int last_buffer_size = 0;
    if( buffer == NULL || buffer_size > last_buffer_size ) {
      buffer = realloc( buffer, buffer_size );
      last_buffer_size = buffer_size;
    }
    glTexImage2D(GL_TEXTURE_2D, 0, internal_format, npo2(width), npo2(height),
                 0, channel_order, format, buffer);
  }
}

void 
LadybugVideo::setupRenderToFbo( int width, int height, int format,
				   GLuint* texture, GLuint *framebuf, GLuint *depthbuf ) 
{
  int texture_width  = npo2(width);
  int texture_height = npo2(height);

  if( *texture  == (GLuint)-1 || 
      *framebuf == (GLuint)-1 || 
      *depthbuf == (GLuint)-1 ) {
    dgc_info("initializing fbo ...\n");
  }

  initTexture( texture, texture_width, texture_height, 4, format,  GL_RGBA );

  /* make framebuffer */
  if( *framebuf == (GLuint)-1 ) {
    glGenFramebuffersEXT(1, framebuf);
    dgc_info("initializing framebuf %2d\n", *framebuf);
  }  

  /* make depth renderbuffer */
  if( *depthbuf == (GLuint)-1 ) {
    glGenRenderbuffersEXT(1, depthbuf);
    glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, *depthbuf);
    glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT, 
                             texture_width, texture_height);
    dgc_info("initializing depthbuf %2d\n", *depthbuf);
  }
    
  /* Render to offscreen FrameBuffer */
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, *framebuf);
  glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, *depthbuf);
  glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT,
                            GL_TEXTURE_2D, *texture, 0);
  glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
                               GL_RENDERBUFFER_EXT, *depthbuf);
}


void 
LadybugVideo::demosaicDrawGeometry( float width_ratio, float height_ratio, 
				    float ox, float oy, float ox2, float oy2) 
{
  ox /= 1024.0; 
  oy /= 1024.0;
  ox2 /= 1024.0; 
  oy2 /= 1024.0;

  glBegin(GL_TRIANGLES);
  {
    glMultiTexCoord2f(GL_TEXTURE0, ox + 0.0f,         oy + 0.0f         ); // Blue 
    glMultiTexCoord2f(GL_TEXTURE1, ox +-1.0f/1024.0f, oy + 0.0f         ); // Green1
    glMultiTexCoord2f(GL_TEXTURE2, ox + 0.0f,         oy +-1.0f/1024.0f ); // Green2
    glMultiTexCoord2f(GL_TEXTURE3, ox +-1.0f/1024.0f, oy +-1.0f/1024.0f ); // Red
    glMultiTexCoord2f(GL_TEXTURE4, ox + 0.0f,         oy + 0.0f         ); // Alpha
    glVertex2f( 0.0f+ox2, 0.0f+oy2);

    glMultiTexCoord2f(GL_TEXTURE0, ox + 2.0f*width_ratio + 0.0f,         oy + 0.0f         ); // Blue 
    glMultiTexCoord2f(GL_TEXTURE1, ox + 2.0f*width_ratio +-1.0f/1024.0f, oy + 0.0f         ); // Green1 
    glMultiTexCoord2f(GL_TEXTURE2, ox + 2.0f*width_ratio + 0.0f,         oy +-1.0f/1024.0f ); // Green2
    glMultiTexCoord2f(GL_TEXTURE3, ox + 2.0f*width_ratio +-1.0f/1024.0f, oy +-1.0f/1024.0f ); // Red
    glMultiTexCoord2f(GL_TEXTURE4, ox + 2.0f*width_ratio + 0.0f,         oy + 0.0f         ); // Alpha
    glVertex2f( 2.0f+ox2, 0.0f+oy2);

    glMultiTexCoord2f(GL_TEXTURE0, ox + 0.0f,          oy + 0.0f         + 2.0*height_ratio ); // Blue
    glMultiTexCoord2f(GL_TEXTURE1, ox +-1.0f/1024.0f,  oy + 0.0f         + 2.0*height_ratio ); // Green1 
    glMultiTexCoord2f(GL_TEXTURE2, ox + 0.0f,          oy +-1.0f/1024.0f + 2.0*height_ratio ); // Green2
    glMultiTexCoord2f(GL_TEXTURE3, ox +-1.0f/1024.0f,  oy +-1.0f/1024.0f + 2.0*height_ratio ); // Red
    glMultiTexCoord2f(GL_TEXTURE4, ox + 0.0f,          oy + 0.0f         + 2.0*height_ratio ); // Alpha
    glVertex2f( 0.0f+ox2, 2.0f+oy2   );
  }
  glEnd();
}

void 
LadybugVideo::demosaicBilinear( int camera ) 
{
  int     width        = distortedImage[camera].width;
  int     height       = distortedImage[camera].height;
  float   width_ratio  = width / (float)npo2(width);
  float   height_ratio = height/ (float)npo2(height);
  static  GLuint  gl_fb = -1, gl_db = -1;

  setupRenderToFbo( width, height, GL_HALF_FLOAT_ARB,
		    (GLuint*)&(texture_id[camera]), 
		    &gl_fb, &gl_db );

  glViewport( 0, 0, width,height );
  vlr::set_display_mode_2D(1,1);
  
  if( cgeffect_demosaic_bilinear == NULL ) {
    cgeffect_demosaic_bilinear = cgCreateEffect(myCgContext, 
						_cg_demosaic_bilinear_code_data,
						NULL);
    checkCgEffect( cgeffect_demosaic_bilinear );
  }  
  int iBayerImage = camera;
  if (config.version==LADYBUG_VERSION_3) {
    cgGLSetupSampler( cgGetNamedEffectParameter( cgeffect_demosaic_bilinear, "texB"  ), 
		      bayer_texture_id[iBayerImage][3] );
    cgGLSetupSampler( cgGetNamedEffectParameter( cgeffect_demosaic_bilinear, "texG1" ), 
		      bayer_texture_id[iBayerImage][1] );
    cgGLSetupSampler( cgGetNamedEffectParameter( cgeffect_demosaic_bilinear, "texG2" ), \
		      bayer_texture_id[iBayerImage][2] );
    cgGLSetupSampler( cgGetNamedEffectParameter( cgeffect_demosaic_bilinear, "texR"  ), 
		      bayer_texture_id[iBayerImage][0] );
  } else { // config.version==LADYBUG_VERSION_2) {
    cgGLSetupSampler( cgGetNamedEffectParameter( cgeffect_demosaic_bilinear, "texB"  ), 
		      bayer_texture_id[iBayerImage][0] );
    cgGLSetupSampler( cgGetNamedEffectParameter( cgeffect_demosaic_bilinear, "texG1" ), 
		      bayer_texture_id[iBayerImage][1] );
    cgGLSetupSampler( cgGetNamedEffectParameter( cgeffect_demosaic_bilinear, "texG2" ), \
		      bayer_texture_id[iBayerImage][2] );
    cgGLSetupSampler( cgGetNamedEffectParameter( cgeffect_demosaic_bilinear, "texR"  ), 
		      bayer_texture_id[iBayerImage][3] );
  }
  static CGpass pass;
  CGtechnique technique = cgGetFirstTechnique( cgeffect_demosaic_bilinear );  
  if( (pass = cgGetFirstPass(technique)) == NULL )
    dgc_die("demosaic_gpu: cannot get pass 0\n");
  
  // Render Pass 1: combine tex* to target
  glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);

  cgSetPassState(pass);
  demosaicDrawGeometry(width_ratio, height_ratio, 0.5, 0.5); // needs to be 0.5,0.5 not -0.5,-0.5
  cgResetPassState(pass);

  /* Restore old Framebuffer */
  glEnable(GL_DEPTH_TEST);
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
  glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, 0);
}

void 
LadybugVideo::initializeCg() 
{
  static bool cg_initialized = false;
  if( !cg_initialized ) {
    cg_initialized = true;
    myCgContext = cgCreateContext();
    checkForCgError("in LadybugVideo::init_cg(): cannot create Cg context");

    cgGLRegisterStates(myCgContext);
    checkForCgError("in LadybugVideo::init_cg(): cannot register GL states");

    myCgFragmentProfile = cgGLGetLatestProfile(CG_GL_FRAGMENT);
    cgGLSetOptimalOptions(myCgFragmentProfile);
    checkForCgError("in LadybugVideo::init_cg(): cannot select fragment profile");

    cgGLSetManageTextureParameters( myCgContext, true );
  }
}

