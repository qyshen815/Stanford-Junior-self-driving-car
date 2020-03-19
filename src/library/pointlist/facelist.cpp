#include <roadrunner.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include "facelist.h"

void dgc_facelist_new(dgc_facelist_p points, int num_points)
{
  points->num_points = num_points;
  points->points = (dgc_polygon_p)calloc(points->num_points, 
                                         sizeof(dgc_polygon_t));
  dgc_test_alloc(points->points);
  points->full = 0;
  points->mark = 0;
}

void dgc_facelist_add_color_points(dgc_facelist_p points, dgc_polygon_p poly,
                                   int n, double max_size)
{
  int i;
  double d1, d2, d3, d4;

  for(i = 0; i < n; i++) 
    if(poly[i].v1.range < 40.0 && poly[i].v2.range < 40.0 &&
       poly[i].v3.range < 40.0 && poly[i].v4.range < 40.0) {
      points->points[points->mark] = poly[i];
      
      d1 = hypot(poly[i].v1.x - poly[i].v2.x, poly[i].v1.y - poly[i].v2.y);
      d2 = hypot(poly[i].v2.x - poly[i].v3.x, poly[i].v2.y - poly[i].v3.y);
      d3 = hypot(poly[i].v3.x - poly[i].v4.x, poly[i].v3.y - poly[i].v4.y);
      d4 = hypot(poly[i].v4.x - poly[i].v1.x, poly[i].v4.y - poly[i].v1.y);
      if(d1 < max_size && d2 < max_size && d3 < max_size && d4 < max_size)
        points->points[points->mark].fill = 1;
      else
        points->points[points->mark].fill = 0;

      points->mark++;
      if(points->mark == points->num_points) {
        points->mark = 0;
        points->full = 1;
      }
    }
}

void dgc_facelist_reset(dgc_facelist_p points)
{
  points->mark = 0;
  points->full = 0;
}

void dgc_facelist_draw_mesh(dgc_facelist_p points, 
                            double origin_x, double origin_y, double origin_z)
{
  int i, max;

  if(points->full || points->mark > 0) {
    max = points->full ? points->num_points : points->mark;

    for(i = 0; i < max; i++)
      if(points->points[i].fill) {
        glBegin(GL_QUADS);
        glColor3f(points->points[i].v1.r,
                  points->points[i].v1.g,
                  points->points[i].v1.b);
        glVertex3f(points->points[i].v1.x - origin_x, 
                   points->points[i].v1.y - origin_y, 
                   points->points[i].v1.z - origin_z);
        
        glColor3f(points->points[i].v2.r,
                  points->points[i].v2.g,
                  points->points[i].v2.b);
        glVertex3f(points->points[i].v2.x - origin_x, 
                   points->points[i].v2.y - origin_y, 
                   points->points[i].v2.z - origin_z);
        
        glColor3f(points->points[i].v3.r,
                  points->points[i].v3.g,
                  points->points[i].v3.b);
        glVertex3f(points->points[i].v3.x - origin_x, 
                   points->points[i].v3.y - origin_y, 
                   points->points[i].v3.z - origin_z);
        
        glColor3f(points->points[i].v4.r,
                  points->points[i].v4.g,
                  points->points[i].v4.b);
        glVertex3f(points->points[i].v4.x - origin_x, 
                   points->points[i].v4.y - origin_y, 
                   points->points[i].v4.z - origin_z);
        
        glEnd();
      }

  }  
}

void dgc_facelist_draw_2D(dgc_facelist_p points, 
                          double origin_x, double origin_y)
{
  int i, max;

  if(points->full || points->mark > 0) {
    max = points->full ? points->num_points : points->mark;

    for(i = 0; i < max; i++)
      if(points->points[i].fill) {
        glBegin(GL_QUADS);
        glColor3f(points->points[i].v1.r,
                  points->points[i].v1.g,
                  points->points[i].v1.b);
        glVertex2f(points->points[i].v1.x - origin_x, 
                   points->points[i].v1.y - origin_y);
        
        glColor3f(points->points[i].v2.r,
                  points->points[i].v2.g,
                  points->points[i].v2.b);
        glVertex2f(points->points[i].v2.x - origin_x, 
                   points->points[i].v2.y - origin_y);
        
        glColor3f(points->points[i].v3.r,
                  points->points[i].v3.g,
                  points->points[i].v3.b);
        glVertex2f(points->points[i].v3.x - origin_x, 
                   points->points[i].v3.y - origin_y);
        
        glColor3f(points->points[i].v4.r,
                  points->points[i].v4.g,
                  points->points[i].v4.b);
        glVertex2f(points->points[i].v4.x - origin_x, 
                   points->points[i].v4.y - origin_y);
        glEnd();
      }

  }  
}

