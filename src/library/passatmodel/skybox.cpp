#include <roadrunner.h>
#include <gl_support.h>
#include <textures.h>

#include "bluebk.h"
#include "bluedn.h"
#include "blueup.h"
#include "blueft.h"
#include "bluelt.h"
#include "bluert.h"

namespace vlr {

static int skybox_initialized = 0;
static GLint skybox_lid;

void generate_skybox(void) {
  dgc_gl_texture_t* skybox_texture[6];

  skybox_lid = glGenLists(1);
  glNewList(skybox_lid, GL_COMPILE);

  skybox_texture[0] = dgc_gl_load_texture_from_bytes(BLUEUP_SIZE, blueup_data, 256, 1);
  skybox_texture[1] = dgc_gl_load_texture_from_bytes(BLUEDN_SIZE, bluedn_data, 256, 1);
  skybox_texture[2] = dgc_gl_load_texture_from_bytes(BLUERT_SIZE, bluert_data, 256, 1);
  skybox_texture[3] = dgc_gl_load_texture_from_bytes(BLUELT_SIZE, bluelt_data, 256, 1);
  skybox_texture[4] = dgc_gl_load_texture_from_bytes(BLUEFT_SIZE, blueft_data, 256, 1);
  skybox_texture[5] = dgc_gl_load_texture_from_bytes(BLUEBK_SIZE, bluebk_data, 256, 1);

  // djoubert187 _at_ hotmail.com
  glEnable(GL_TEXTURE_2D);
  glColor3f(1, 1, 1);

  // If you have border issues change this to 1.005f
  float r = -1.0;

  // Common Axis Z - FRONT Side
  glBindTexture(GL_TEXTURE_2D, skybox_texture[4]->texture_id);
  glBegin(GL_QUADS);
  glTexCoord2f(1, 0);
  glVertex3f(-r, 1.0, -r);
  glTexCoord2f(1, -1);
  glVertex3f(-r, 1.0, r);
  glTexCoord2f(0, -1);
  glVertex3f(r, 1.0, r);
  glTexCoord2f(0, 0);
  glVertex3f(r, 1.0, -r);
  glEnd();

  // Common Axis Z - BACK side
  glBindTexture(GL_TEXTURE_2D, skybox_texture[5]->texture_id);
  glBegin(GL_QUADS);
  glTexCoord2f(1, 0);
  glVertex3f(-r, -1.0, -r);
  glTexCoord2f(1, -1);
  glVertex3f(-r, -1.0, r);
  glTexCoord2f(0, -1);
  glVertex3f(r, -1.0, r);
  glTexCoord2f(0, 0);
  glVertex3f(r, -1.0, -r);
  glEnd();

  // Common Axis X - Left side
  glBindTexture(GL_TEXTURE_2D, skybox_texture[3]->texture_id);
  glBegin(GL_QUADS);
  glTexCoord2f(1, -1);
  glVertex3f(-1.0, -r, r);
  glTexCoord2f(0, -1);
  glVertex3f(-1.0, r, r);
  glTexCoord2f(0, 0);
  glVertex3f(-1.0, r, -r);
  glTexCoord2f(1, 0);
  glVertex3f(-1.0, -r, -r);
  glEnd();

  // Common Axis X - Right side
  glBindTexture(GL_TEXTURE_2D, skybox_texture[2]->texture_id);
  glBegin(GL_QUADS);
  glTexCoord2f(1, -1);
  glVertex3f(1.0, -r, r);
  glTexCoord2f(0, -1);
  glVertex3f(1.0, r, r);
  glTexCoord2f(0, 0);
  glVertex3f(1.0, r, -r);
  glTexCoord2f(1, 0);
  glVertex3f(1.0, -r, -r);
  glEnd();

  // Common Axis Y - Draw Up side
  glBindTexture(GL_TEXTURE_2D, skybox_texture[0]->texture_id);
  glBegin(GL_QUADS);
  glTexCoord2f(0, 0);
  glVertex3f(r, -r, 1);
  glTexCoord2f(-1, 0);
  glVertex3f(r, r, 1);
  glTexCoord2f(-1, -1);
  glVertex3f(-r, r, 1);
  glTexCoord2f(0, -1);
  glVertex3f(-r, -r, 1);
  glEnd();

  // Common Axis Y - Down side
  glBindTexture(GL_TEXTURE_2D, skybox_texture[1]->texture_id);
  glBegin(GL_QUADS);
  glTexCoord2f(0, 0);
  glVertex3f(r, -r, -1);
  glTexCoord2f(1, 0);
  glVertex3f(r, r, -1);
  glTexCoord2f(1, -1);
  glVertex3f(-r, r, -1);
  glTexCoord2f(0, -1);
  glVertex3f(-r, -r, -1);
  glEnd();
  glDisable(GL_TEXTURE_2D);

  glEndList();
}

void draw_skybox(double r, double z_flat) {
  if (!skybox_initialized) {
    generate_skybox();
    skybox_initialized = 1;
  }

  glPushMatrix();
  glScalef(r, r, r);
  glDisable(GL_LIGHTING);
  glCallList(skybox_lid);
  glPopMatrix();

  glColor3f(0.5, 0.5, 0.5);
  glBegin(GL_QUADS);
  glVertex3f(-r, -r, z_flat);
  glVertex3f(r, -r, z_flat);
  glVertex3f(r, r, z_flat);
  glVertex3f(-r, r, z_flat);
  glEnd();
}

} // namespace vlr
