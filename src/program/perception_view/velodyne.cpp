#include "view.h"

int                 num_pts[NUM_LASER_BEAMS];
velodyne_pt_t       pts[NUM_LASER_BEAMS][MAX_NUM_POINTS_PER_BEAM];

#define    VELO_BLIND_SPOT_START    17000
#define    VELO_BLIND_SPOT_STOP     19000
#define    VELO_NUM_TICKS           36000
#define    BINS_PER_REV             720

void
draw_velodyne( int num_scans, dgc_velodyne_scan_p scans,
    ApplanixPose *cur_pose )
{
  dgc_pose_t             pose_diff;
  int                    n, i, j, l, lb,le, points_drawn = 0;
  dgc_rgb_t              color;
  double                 val;
  float                  v;

  if (num_scans>0) {

    if (show_upper_block) lb = 0;  else lb = 32;
    if (show_lower_block) le = 64; else le = 32;

    glPushMatrix();
    if (plain_mode)
      glTranslatef(0.0, 0.0, -DGC_PASSAT_HEIGHT);

    if (show_lines) {

      // *******************************************************************
      // *    Set beams to individual laser units
      // *******************************************************************
      for (l=0; l<NUM_LASER_BEAMS; l++) {
        num_pts[l] = 0;
      }

      for( i = 0; i < num_scans; i++) {
//        if ((scans[i].encoder > VELO_BLIND_SPOT_START) && (scans[i].encoder < VELO_BLIND_SPOT_STOP))
//          continue;
        pose_diff.x = scans[i].robot.x-cur_pose->smooth_x;
        pose_diff.y = scans[i].robot.y-cur_pose->smooth_y;
        pose_diff.z = scans[i].robot.z-cur_pose->smooth_z;
        for(j = 0; j < 32; j++) {
          l = j + scans[i].block * 32;
          n = num_pts[l];
          if (n<MAX_NUM_POINTS_PER_BEAM) {
            pts[l][n].x = scans[i].p[j].x*0.01+pose_diff.x;
            pts[l][n].y = scans[i].p[j].y*0.01+pose_diff.y;
            pts[l][n].z = scans[i].p[j].z*0.01+pose_diff.z;
            if (scans[i].p[j].range>100)
              pts[l][n].v = 1;
            else
              pts[l][n].v = 0;
            pts[l][n].i = scans[i].p[j].intensity;
            num_pts[l]++;
          }
        }
      }

      // *******************************************************************
      // *    Draw Lines
      // *******************************************************************
      for(l = lb; l < le; l++) {
        for(i = 0; i < num_pts[l]-1; i++) {
          if (show_intensity) {
            //	    v = 0.5 + (pts[l][i].i-40)/80.0;
            //	    if (v<0) v = 0;
            //	    if (v>1) v = 1;
            v = pts[l][i].i/255.0;
            glColor3f( v, v, v );
          } else if (mark_single_beam) {
            if (l==marked_beam) {
              glColor3f( 1.0, 0.0, 0.0 );
            } else {
              glColor3f( 1.0, 1.0, 1.0 );
            }
          } else if (show_colors) {
            glColor3f( crgb[l%NUM_COLORS].r,
                crgb[l%NUM_COLORS].g,
                crgb[l%NUM_COLORS].b );
          }
          if (veloconfig->laser_enabled[l] &&
              pts[l][i].v && pts[l][i+1].v &&
              pts_dist(pts[l][i], pts[l][i+1])<2.5) {
            glBegin(GL_LINES);
            myglVertex3f(pts[l][i].x, pts[l][i].y, pts[l][i].z );
            myglVertex3f(pts[l][i+1].x, pts[l][i+1].y, pts[l][i+1].z );
            glEnd();
            points_drawn++;
          }
        }
      }

    } else {

      // *******************************************************************
      // *    Draw the Points
      // *******************************************************************

      glBegin(GL_POINTS);

      for( i = 0; i < num_scans; i++) {
        pose_diff.x = scans[i].robot.x-cur_pose->smooth_x;
        pose_diff.y = scans[i].robot.y-cur_pose->smooth_y;
        pose_diff.z = scans[i].robot.z-cur_pose->smooth_z;
        for(j = 0; j < 32; j++) {
          l = j + scans[i].block * 32;
          if ( scans[i].p[j].range>0 && veloconfig->laser_enabled[l] &&
              ( (l <  32 && show_upper_block) ||
                  (l >= 32 && show_lower_block) ) ) {
            if (beam_active[l]) {
              if (mark_single_beam) {
                if (l==marked_beam) {
                  glColor3f( 1.0, 0.0, 0.0 );
                } else {
                  glColor3f( 1.0, 1.0, 1.0 );
                }
              } else if (show_intensity) {
                v = scans[i].p[j].intensity / 255.0;
                //v = 0.5 + (scans[i].p[j].intensity-40)/80.0;
                //if (v<0) v = 0;
                //if (v>1) v = 1;
                glColor3f( v, v, v );
              } else if (show_colors==1) {
                glColor3f( crgb[l%NUM_COLORS].r,
                    crgb[l%NUM_COLORS].g,
                    crgb[l%NUM_COLORS].b );
              } else if (show_colors==2) {

//                val = (float(i) / num_scans);
//                int e = scans[i].encoder;
                int offset = (int) (VELO_NUM_TICKS*(-veloconfig->rot_angle[l]/(2*M_PI)));
                int e = scans[i].encoder + offset;
                e = (e < 0) ? (e + VELO_NUM_TICKS) : ((e > VELO_NUM_TICKS) ? (e - VELO_NUM_TICKS) : e);
                int b = (int)floor(e / ((float)VELO_NUM_TICKS/(float)BINS_PER_REV));
                b = b % 10;
                val = ((float)b / 10.0f);

                if (val>1.0) val=1.0;
                if (val<0.0) val=0.0;
                color = dgc_val_to_rgb(val);
                glColor3f( color.r, color.g, color.b );
                if ((e > VELO_BLIND_SPOT_START) && (e < VELO_BLIND_SPOT_STOP))
                  glColor3f( 1.0, 0, 0);
//                else {
//                  val = (scans[i].p[j].z * 0.01/2.0)+0.7;
//                  if (val>1.0) val=1.0;
//                  if (val<0.0) val=0.0;
//                  color = dgc_val_to_rgb(val);
//                  glColor3f( color.r, color.g, color.b );
//                }
              }
              l = j + scans[i].block * 32;
              myglVertex3f(scans[i].p[j].x*0.01+pose_diff.x,
                  scans[i].p[j].y*0.01+pose_diff.y,
                  scans[i].p[j].z*0.01+pose_diff.z);
              points_drawn++;
            }
          }
        }
      }
      glEnd();
    } // end of else draw_lines
    glPopMatrix();

  }
}
