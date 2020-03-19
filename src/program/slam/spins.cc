#include <roadrunner.h>
#include <lltransform.h>
#include <gui3D.h>
#include <vector>
#include "slam_inputs.h"
#include "spins.h"
#include "intensitycal.h"

using namespace std;
using namespace dgc;
using namespace vlr;

SelectedSpin::SelectedSpin()
{
  MapCell *default_cell = new MapCell;

  snum_ = -1;
  valid_ = false;
  default_cell->intensity_sum = 0;
  default_cell->intensity_count = 0;
  grid_ = dgc_grid_initialize(0.15, 250, 250, sizeof(MapCell), default_cell);

  texturedata_ = NULL;
  texture_ = NULL;
  texture_valid_ = false;
}

void SelectedSpin::Select(SlamLogfile *log, int snum)
{
  double applanix_lat, applanix_lon, applanix_alt;
  int si, i, j, beam_num, ring_num;
  SpinPoint p;
  dgc_velodyne_spin s;

  point_.clear();

  log_ = log;
  snum_ = snum;
  valid_ = true;
  pthread_mutex_lock(log->mutex());
  spin_.load(log->fp(), log->velodyne_config(), log->index(),
	     snum, &applanix_lat, &applanix_lon, &utm_z_);
  latLongToUtm(applanix_lat, applanix_lon, &utm_x_, &utm_y_, utmzone_);
  smooth_x_ = log->index()->spin[snum].pose[0].smooth_x;
  smooth_y_ = log->index()->spin[snum].pose[0].smooth_y;
  smooth_z_ = log->index()->spin[snum].pose[0].smooth_z;
  texture_valid_ = false;

  for (si = max(snum - 5, 0); 
       si < min(snum + 5, log->index()->num_spins - 1); si++) {
    s.load(log->fp(), log->velodyne_config(), log->index(),
	   si, &applanix_lat, &applanix_lon, &applanix_alt);

    for(i = 0; i < s.num_scans; i++) 
      for(j = 0; j < VELO_BEAMS_IN_SCAN; j++) {
	if(s.scans[i].p[j].range < 0.01)
	  continue;
	beam_num = j + s.scans[i].block * 32;
	ring_num = log_->velodyne_config()->inv_beam_order[beam_num];
	
	p.i =
	  (log_->intensity_calibration()->Lookup(ring_num,
						 s.scans[i].p[j].intensity)
	     - 20) * 3 / 255.0;
	if(p.i < 0) p.i = 0;
	if(p.i > 1) p.i = 1;
	p.x = s.scans[i].p[j].x * 0.01 + s.scans[i].robot.x;
	p.y = s.scans[i].p[j].y * 0.01 + s.scans[i].robot.y;
	p.z = s.scans[i].p[j].z * 0.01 + s.scans[i].robot.z;
	point_.push_back(p);
      }

    
  }
  
  pthread_mutex_unlock(log->mutex());

}

void SelectedSpin::FillRollingGrid(double travel_dist)
{
  int i, j, k, beam_num, ring_num, start_spin, end_spin;
  double lat, lon, alt, u, x, y, z, speed;
  dgc_velodyne_index_pose *pose;
  dgc_velodyne_spin spin;
  MapCell *cell;

  pose = &(log_->index()->spin[snum_].pose[0]);
  dgc_grid_clear(grid_);
  dgc_grid_recenter_grid(grid_, pose->smooth_x, pose->smooth_y);

  i = snum_;
  while (i >= 0 && 
	 hypot(pose->smooth_x - log_->index()->spin[i].pose[0].smooth_x,
	       pose->smooth_y - log_->index()->spin[i].pose[0].smooth_y)
	 < travel_dist / 2.0)
    i--;
  if (i < 0)
    i = 0;
  start_spin = i;
  i = snum_;
  while (i < log_->index()->num_spins &&
	 hypot(pose->smooth_x - log_->index()->spin[i].pose[0].smooth_x,
	       pose->smooth_y - log_->index()->spin[i].pose[0].smooth_y) 
	 < travel_dist / 2.0)
    i++;
  if (i >= log_->index()->num_spins)
    i = log_->index()->num_spins - 1;
  end_spin = i;

  for (i = start_spin; i <= end_spin; i++) {
    speed = hypot(log_->index()->spin[i].pose[0].v_east,
		  log_->index()->spin[i].pose[0].v_north);
    if (speed < dgc_mph2ms(1.0))
	continue;
    
    spin.load(log_->fp(), log_->velodyne_config(), log_->index(), 
	      i, &lat, &lon, &alt);
    
    for (j = 0; j < spin.num_scans; j++) 
      for(k = 0; k < VELO_BEAMS_IN_SCAN; k++) {
	if(spin.scans[j].p[k].range < 0.01)
	  continue;
	beam_num = k + spin.scans[j].block * 32;
	ring_num = log_->velodyne_config()->inv_beam_order[beam_num];
	
	if (ring_num == 32 || ring_num == 34 || ring_num == 36)
	  continue;
	
	u = log_->intensity_calibration()->Lookup(ring_num, 
						  spin.scans[j].p[k].intensity);
	
	x = spin.scans[j].p[k].x * 0.01 + spin.scans[j].robot.x;
	y = spin.scans[j].p[k].y * 0.01 + spin.scans[j].robot.y;
	z = spin.scans[j].p[k].z * 0.01 + spin.scans[j].robot.z;

	cell = (MapCell *)dgc_grid_get_xy(grid_, x, y);
	if (cell != NULL) {
	  cell->intensity_sum += u;
	  cell->intensity_count++;
	}
      }
  }
}

void RollingGridToTextureData(dgc_grid_p grid, unsigned char *texturedata)
{
  MapCell *cell;
  int i, j;
  double u;

  for(j = 0; j < grid->rows; j++) 
    for (i = 0; i < grid->cols; i++) {
      cell = (MapCell *)dgc_grid_get_rc_local(grid, j, i);
      if (cell->intensity_count == 0)
	u = 0;
      else {
	u = cell->intensity_sum / (double)cell->intensity_count;
	u = (u - 20.0) * 3;
	if(u < 0)
	  u = 0;
	if(i > 255)
	  u = 255;
      }
      texturedata[(j * grid->cols + i) * 3] = (unsigned char)u;
      texturedata[(j * grid->cols + i) * 3 + 1] = (unsigned char)u;
      texturedata[(j * grid->cols + i) * 3 + 2] = (unsigned char)u;
    }
}

void SelectedSpin::UpdateIntensityGrid(void)
{
  if (!valid())
    return;

  FillRollingGrid(30);

  if (texture_ == NULL) {
    texture_ = dgc_gl_empty_texture(grid_->cols, grid_->rows,
				    1024, 0);
    texturedata_ = new unsigned char[grid_->cols * grid_->rows * 3];
  }
  RollingGridToTextureData(grid_, texturedata_);
  dgc_gl_update_texture_from_raw(texture_, texturedata_, 
				 grid_->cols, grid_->rows);
  texture_valid_ = true;
}

void draw_texture_transparent(dgc_gl_texture_t* texture, double x1, double y1,
			      double x2, double y2, int smooth, 
			      double alpha)
{
  double dx, dy;

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, texture->texture_id);

  if(smooth) {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    dx = 1 / (2 * texture->texture_width);
    dy = 1 / (2 * texture->texture_height);
  }
  else {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    dx = 0;
    dy = 0;
  }

  glColor4f(1, 1, 1, alpha);
  glBegin(GL_POLYGON);
  glTexCoord2f(dx, dy);
  glVertex2f(x1, y1);
  glTexCoord2f(texture->max_u - dx, dy);
  glVertex2f(x2, y1);
  glTexCoord2f(texture->max_u - dx, texture->max_v - dy);
  glVertex2f(x2, y2);
  glTexCoord2f(dx, texture->max_v - dy);
  glVertex2f(x1, y2);
  glEnd();
  glDisable(GL_TEXTURE_2D);
}

void SelectedSpin::DrawIntensityGrid(double origin_x, double origin_y,
				     double origin_z, double alpha)
{
  double w, h;

  glPushMatrix();
  glTranslatef(0, 0, -origin_z);
  if (texture_ != NULL) {
    w = grid_->cols * grid_->resolution;
    h = grid_->rows * grid_->resolution;
    draw_texture_transparent(texture_, 
			     utm_x_ - origin_x - w / 2.,
			     utm_y_ - origin_y - h / 2.,
			     utm_x_ - origin_x + w / 2.,
			     utm_y_ - origin_y + h / 2., 1, alpha);
  }
  glPopMatrix();
}

void SelectedSpin::DrawSpin(double origin_x, double origin_y,
			    double origin_z, dgc_transform_t t)
{
  int i;
  double x, y, z;

  if (!valid())
    return;

  glBegin(GL_POINTS);
  for(i = 0; i < (int)point_.size(); i++) {
    glColor3f(point_[i].i, point_[i].i, point_[i].i);
    x = point_[i].x;
    y = point_[i].y;
    z = point_[i].z;
    dgc_transform_point(&x, &y, &z, t);
    glVertex3f(x - origin_x, y - origin_y, z - origin_z);
  }
  glEnd();
}


