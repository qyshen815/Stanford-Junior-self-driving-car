#include <roadrunner.h>
#include <gui3D.h>
#include <textures.h>
#include <gl_support.h>

#include "signs.h"

using namespace vlr;

static dgc_gl_texture_t* accident_sign = NULL;
static dgc_gl_texture_t* death_sign = NULL;
static dgc_gl_texture_t* slippery_sign = NULL;
static dgc_gl_texture_t* warning_sign = NULL;

void draw_accident_sign_3D(double x, double y, double z, 
			   double r, double heading, double blend)
{
  /* load sign textures */
  if(accident_sign == NULL) {
    accident_sign = dgc_gl_load_texture_from_bytes(ACCIDENT_SIGN_SIZE, 
						   accident_sign_data,
						   256, 1);
    if(accident_sign == NULL)
      dgc_die("Error: could not load accident sign image.\n");
  }

  glPushMatrix(); 
  {
    glTranslatef(x, y, z);
    
    glRotatef(dgc_r2d(heading) - 90.0, 0, 0, 1);
    
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, accident_sign->texture_id);
    
    /* scale the texture */
    glMatrixMode(GL_TEXTURE);

    glPushMatrix(); 
    {
      glScalef(accident_sign->max_u, accident_sign->max_v, 1);
      
      glColor4f(1, 1, 1, blend);
      glBegin(GL_POLYGON); {
	glTexCoord2f( 0.0, 1.0 );
	glVertex2f( 2.0*r, -0.5 );

	glTexCoord2f( 1.0, 1.0 );
	glVertex2f( -2.0*r, -0.5 );

	glTexCoord2f( 1.0, 0.0 );
	glVertex2f( -2.0*r, -2.22*r-0.5 );

	glTexCoord2f( 0.0, 0.0 );
	glVertex2f( 2.0*r, -2.22*r-0.5 );
      }
      glEnd();
      
      /* undo the scaling */
    } 
    glPopMatrix();

    glDisable(GL_TEXTURE_2D);
    glMatrixMode(GL_MODELVIEW);
    
  } 
  glPopMatrix();
}

void draw_death_sign_3D(double x, double y, double z, 
			double r, double heading, double blend)
{
  /* load sign textures */
  if(death_sign == NULL) {
    death_sign = dgc_gl_load_texture_from_bytes(DEATH_SIGN_SIZE, 
						death_sign_data,
						256, 1);
    if(death_sign == NULL)
      dgc_die("Error: could not load death sign image.\n");
  }
  
  glPushMatrix(); {
    glTranslatef(x, y, z);
    
    glRotatef(dgc_r2d(heading) - 90.0, 0, 0, 1);
    
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, death_sign->texture_id);
    
    /* scale the texture */
    glMatrixMode(GL_TEXTURE);
    
    glPushMatrix(); {
      glScalef(death_sign->max_u, death_sign->max_v, 1);
    
      glColor4f(1, 1, 1, blend);
      glBegin(GL_POLYGON); {
	glTexCoord2f( 0.5, 1.0-0.05 );
	glVertex2f( 0.0, -0.5 );
	
	glTexCoord2f( 0.05, 0.05 );
	glVertex2f( -1.25*r, -2.25*r-0.5 );

	glTexCoord2f( 1.0-0.05, 0.05 );
	glVertex2f( 1.25*r, -2.25*r-0.5 );
      }
      glEnd();
      
      /* undo the scaling */
    } glPopMatrix();
    glDisable(GL_TEXTURE_2D);

    glMatrixMode(GL_MODELVIEW);
    
  } glPopMatrix();
}

void draw_slippery_sign_3D(double x, double y, double z, 
			   double r, double heading, double blend)
{
  /* load sign textures */
  if(slippery_sign == NULL) {
    slippery_sign = dgc_gl_load_texture_from_bytes(SLIPPERY_SIGN_SIZE, 
						   slippery_sign_data,
						   256, 1);
    if(slippery_sign == NULL)
      dgc_die("Error: could not load slippery sign image.\n");
  }
  
  glPushMatrix(); {
    glTranslatef(x, y, z);
    
    glRotatef(dgc_r2d(heading) - 90.0, 0, 0, 1);
    
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, slippery_sign->texture_id);
    
    /* scale the texture */
    glMatrixMode(GL_TEXTURE);
    
    glPushMatrix(); {
      glScalef(slippery_sign->max_u, slippery_sign->max_v, 1);
    
      glColor4f(1, 1, 1, blend);
      glBegin(GL_POLYGON); {
	glTexCoord2f( 0.5, 1.0-0.05 );
	glVertex2f( 0.0, -0.5 );
	
	glTexCoord2f( 0.05, 0.05 );
	glVertex2f( -1.25*r, -2.2277*r-0.5 );

	glTexCoord2f( 1.0-0.05, 0.05 );
	glVertex2f( 1.25*r, -2.2277*r-0.5 );
      }
      glEnd();
      
      /* undo the scaling */
    } glPopMatrix();
    glDisable(GL_TEXTURE_2D);

    glMatrixMode(GL_MODELVIEW);
    
  } glPopMatrix();
}

void draw_warning_sign_3D(double x, double y, double z, 
			  double r, double heading, double blend)
{
  /* load sign textures */
  if(warning_sign == NULL) {
    warning_sign = dgc_gl_load_texture_from_bytes(WARNING_SIGN_SIZE, 
						  warning_sign_data,
						  256, 1);
    if(warning_sign == NULL)
      dgc_die("Error: could not load warning sign image.\n");
  }
  
  glPushMatrix(); {
    glTranslatef(x, y, z);
    
    glRotatef(dgc_r2d(heading) - 90.0, 0, 0, 1);
    
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, warning_sign->texture_id);
    
    /* scale the texture */
    glMatrixMode(GL_TEXTURE);
    
    glPushMatrix(); {
      glScalef(warning_sign->max_u, warning_sign->max_v, 1);
    
      glColor4f(1, 1, 1, blend);
      glBegin(GL_POLYGON); {
	glTexCoord2f( 0.5, 1.0-0.05 );
	glVertex2f( 0.0, -0.5 );
	
	glTexCoord2f( 0.05, 0.05 );
	glVertex2f( -1.25*r, -2.090625*r-0.5 );

	glTexCoord2f( 1.0-0.05, 0.05 );
	glVertex2f( 1.25*r, -2.090625*r-0.5 );
      }
      glEnd();
      
      /* undo the scaling */
    } glPopMatrix();
    glDisable(GL_TEXTURE_2D);

    glMatrixMode(GL_MODELVIEW);
    
  } glPopMatrix();
}

