#include <roadrunner.h>
#include <GL/gl.h>
#include <GL/glut.h>

void renderStrokeString(float x, float y, float z, void *font, float size, 
                        char *string)
{  
  char *c;

  glPushMatrix();
  glTranslatef(x, y, z);
  glScalef(size / 100.0, size / 100.0, size / 100.0);
  for(c = string; *c != '\0'; c++)
    glutStrokeCharacter(font, *c);
  glPopMatrix();
}

GLint generate_laser(void)
{
  GLint laser = glGenLists(1);
  int i;
  float angle;

  GLfloat casing_ambient[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat casing_diffuse[] = { 0.92, 0.91, 0.66, 1.0 };
  GLfloat casing_specular[] = { 0.25, 0.25, 0.25, 1.0 };
  GLfloat casing_emission[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat casing_phExp = 0.0;

  GLfloat visor_ambient[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat visor_diffuse[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat visor_specular[] = { 0.5, 0.5, 0.5, 1.0 };
  GLfloat visor_emission[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat visor_phExp = 32.0;

  glNewList(laser, GL_COMPILE);
  
  glPushMatrix();
  
  /* define the casing material */
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, casing_ambient);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, casing_diffuse);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, casing_specular);
  glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, casing_emission);
  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, casing_phExp);

  glScalef( 0.185/0.15, 0.156/0.15, 0.210/0.18 );

  glBegin(GL_QUADS);
  glNormal3f(0, 0, -1.0);
  glVertex3f(-0.075, -0.075, -0.09);
  glVertex3f(-0.075, 0.075, -0.09);
  glVertex3f(0.075, 0.075, -0.09);
  glVertex3f(0.075, -0.075, -0.09);
  glNormal3f(0, 0, 1.0);
  glVertex3f(-0.075, -0.075, -0.08);
  glVertex3f(-0.075, 0.075, -0.08);
  glVertex3f(0.075, 0.075, -0.08);
  glVertex3f(0.075, -0.075, -0.08);
  glNormal3f(0, -1, 0);
  glVertex3f(-0.02, -0.075, -0.09);
  glVertex3f(0.075, -0.075, -0.09);
  glVertex3f(0.075, -0.075, -0.08);
  glVertex3f(-0.02, -0.075, -0.08);
  glNormal3f(0, 1, 0);
  glVertex3f(-0.02, 0.075, -0.09);
  glVertex3f(0.075, 0.075, -0.09);
  glVertex3f(0.075, 0.075, -0.08);
  glVertex3f(-0.02, 0.075, -0.08);
  glNormal3f(1, 0, 0);
  glVertex3f(0.075, -0.075, -0.09);
  glVertex3f(0.075, 0.075, -0.09);
  glVertex3f(0.075, 0.075, -0.08);
  glVertex3f(0.075, -0.075, -0.08);
  glNormal3f(0, 0, 1.0);
  glVertex3f(-0.075, -0.075, 0.09);
  glVertex3f(-0.075, 0.075, 0.09);
  glVertex3f(0.075, 0.075, 0.09);
  glVertex3f(0.075, -0.075, 0.09);
  glNormal3f(0, 0, -1.0);
  glVertex3f(-0.075, -0.075, 0.02);
  glVertex3f(-0.075, 0.075, 0.02);
  glVertex3f(0.075, 0.075, 0.02);
  glVertex3f(0.075, -0.075, 0.02);
  glNormal3f(-1.0, 0, 0);
  glVertex3f(-0.075, -0.075, -0.09);
  glVertex3f(-0.075, 0.075, -0.09);
  glVertex3f(-0.075, 0.075, 0.09);
  glVertex3f(-0.075, -0.075, 0.09);
  glNormal3f(0, -1, 0);
  glVertex3f(-0.075, -0.075, 0.02);
  glVertex3f(0.075, -0.075, 0.02);
  glVertex3f(0.075, -0.075, 0.09);
  glVertex3f(-0.075, -0.075, 0.09);
  glNormal3f(0, 1, 0);
  glVertex3f(-0.075, 0.075, 0.02);
  glVertex3f(0.075, 0.075, 0.02);
  glVertex3f(0.075, 0.075, 0.09);
  glVertex3f(-0.075, 0.075, 0.09);
  glNormal3f(1, 0, 0);
  glVertex3f(0.075, -0.075, 0.02);
  glVertex3f(0.075, 0.075, 0.02);
  glVertex3f(0.075, 0.075, 0.09);
  glVertex3f(0.075, -0.075, 0.09);
  glNormal3f(0, -1, 0);
  glVertex3f(-0.075, -0.075, -0.09);
  glVertex3f(-0.02, -0.075, -0.09);
  glVertex3f(-0.02, -0.075, 0.02);
  glVertex3f(-0.075, -0.075, 0.02);
  glNormal3f(0, 1, 0);
  glVertex3f(-0.075, 0.075, -0.09);
  glVertex3f(-0.02, 0.075, -0.09);
  glVertex3f(-0.02, 0.075, 0.02);
  glVertex3f(-0.075, 0.075, 0.02);
  glEnd();

  /* define the visor material */
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, visor_ambient);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, visor_diffuse);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, visor_specular);
  glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, visor_emission);
  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, visor_phExp);
  
  glBegin(GL_QUAD_STRIP);
  for(i = 0; i <= 6; i++) {
    angle = -M_PI_2 + i / 6.0 * M_PI;
    glNormal3f(cos(angle), sin(angle), 0.0);
    glVertex3f(-0.02 + 0.07 * cos(angle), 0 + 0.07 * sin(angle), -0.08);
    glVertex3f(-0.02 + 0.07 * cos(angle), 0 + 0.07 * sin(angle), 0.02);
  }
  glEnd();

  /* draw text label */
  glPushMatrix();
  glColor3f(0, 0, 1);
  glTranslatef(0.08, -0.06, .03);
  glRotatef(90, 0, 1, 0);
  glRotatef(90, 0, 0, 1);
  glLineWidth(2.0);
  glDisable(GL_LIGHTING);
  renderStrokeString(0, 0, 0, GLUT_STROKE_ROMAN, 0.045, "SICK");
  glEnable(GL_LIGHTING);
  glPopMatrix();
  glPopMatrix();
  glEndList();
  return laser;
}
