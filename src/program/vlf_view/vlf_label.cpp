#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <velo_support.h>
#include <imagery.h>
#include <gui3D.h>
#include <lltransform.h>
#include <transform.h>
#include <passat_constants.h>
#include <param_interface.h>
#include <passatmodel.h>
#include <car_list.h>
#include <vector>

#define LEFT	1	
#define	RIGHT	2
#define FRONT	3	
#define BACK	4	

using namespace dgc;
using namespace vlr;
using std::vector;

car_list_t carlist;

int align_car(void);

float wrap(float t) {
  while(t > M_PI)
    t -= 2*M_PI;
  while(t < -M_PI)
    t += 2*M_PI;
  return t;
}

void draw_car(car_t* car, double origin_x, double origin_y, int t)
{
  double x, y, theta;
  TurnSignalState signal;
  bool extrapolated;
  int i;
  double angle;
  double l = car->l;
  double w = car->w;

  if(!car->estimate_signal(t, &signal))
    return;
  if(!car->estimate_pose(t, &x, &y, &theta, &extrapolated))
    return;

  if(car->fixed)
    glColor3f(0.4, 0.6, 0.9);
  else if(extrapolated)
    glColor4f(1, 1, 0, 0.25);
  else
    glColor3f(1, 1, 0);

  glPushMatrix();
  glTranslatef(x - origin_x, y - origin_y, 0);
  glRotatef(dgc_r2d(theta), 0, 0, 1);

  glBegin(GL_POLYGON);
  glVertex2f(l / 2, w / 2);
  glVertex2f(l / 2, -w / 2);
  glVertex2f(-l / 2, -w / 2);
  glVertex2f(-l / 2, w / 2);
  glEnd();

  glColor3f(0, 0, 0);
  glBegin(GL_LINE_LOOP);
  glVertex2f(l / 2, w / 2);
  glVertex2f(l / 2, -w / 2);
  glVertex2f(-l / 2, -w / 2);
  glVertex2f(-l / 2, w / 2);
  glEnd();
  glBegin(GL_LINES);
  glVertex2f(l / 2 * 0.5, 0);
  glVertex2f(l / 2 * 1.5, 0);
  glEnd();
  
  glColor3f(1.0, 0.0, 0.0);
  if (signal == 0) {
	  draw_circle(-l/2, 0, 0.5, 1);
  }
  else if (signal == 1) {
	  draw_circle(-l/2, w/2, 0.5, 1);
  }
  else if (signal == 2) {
	  draw_circle(-l/2, -w/2, 0.5, 1);
  }

  glBegin(GL_LINE_LOOP);
  for(i = 0; i < 50; i++) {
    angle = i / 50.0 * M_PI * 2;
    glVertex3f(cos(angle) * 0.2, 
        sin(angle) * 0.2, 0.0);
  }
  glEnd();

  glBegin(GL_LINE_LOOP);
  for(i = 0; i < 50; i++) {
    angle = i / 50.0 * M_PI * 2;
    glVertex3f(l / 2 + cos(angle) * 0.2, sin(angle) * 0.2, 0.0);
  }
  glEnd();

  glBegin(GL_LINE_LOOP);
  for(i = 0; i < 50; i++) {
    angle = i / 50.0 * M_PI * 2;
    glVertex3f(l / 2 + cos(angle) * 0.2, -w / 2 + sin(angle) * 0.2, 0.0);
  }
  glEnd();

  glPopMatrix();
}

/* parameters */

char *imagery_root;
char *cal_filename = NULL;
dgc_transform_t velodyne_offset;
char labels_filename[300], simple_labels_filename[300];

/* velodyne stuff */

dgc_velodyne_file_p velodyne_file = NULL;
dgc_velodyne_index velodyne_index;
dgc_velodyne_config_p velodyne_config = NULL;
dgc_velodyne_spin spin, hold_spin;

int current_spin_num = 0, hold_spin_num = -1;
double current_time;

/* graphics */

dgc_passatwagonmodel_t* passat = NULL;
int large_points = 1;
int color_mode = 2;
int draw_flat = 0;
int last_mouse_x = 0, last_mouse_y = 0;

#define     MODE_NONE                0
#define     MODE_SELECT_CENTER       1
#define     MODE_SELECT_FRONT        2
#define     MODE_SELECT_SIDE         3
#define     MODE_SELECT_CORNER       4

int edit_mode = MODE_NONE;
int edit_car_num = -1;
double last_car_theta = 0;
int last_car_num = -1;

void keyboard(unsigned char key, int x, int y)
{
  double applanix_lat, applanix_lon, applanix_alt, scene_x, scene_y;
  int delta = 0, i, n;
  car_pose_t *p, p2;

  /* figure out where the user clicked on the plane */
  gui3D_pick_point(x, y, &scene_x, &scene_y);
  scene_x += velodyne_index.spin[current_spin_num].pose[0].smooth_x;
  scene_y += velodyne_index.spin[current_spin_num].pose[0].smooth_y;

  if(key >= '1' && key <= '9')
    delta = key - '0';
  else
    switch(key) {
    case 'i': case 'I':
      dgc_imagery_cycle_imagery_type();
      break;
    case 27: case 'q': case 'Q':
      carlist.save_labels(labels_filename);
      carlist.save_labels_simple(simple_labels_filename, 
          velodyne_index.num_spins);
      exit(0);
      break;
    case 'a':
      align_car();
      break;
    case 'e':
      carlist.rotate_car(M_PI);
      break;
    case 'w':
      carlist.rotate_car(M_PI/4);
      break;
    case 'r':
      carlist.rotate_car(-M_PI/4);
      break;
    case 's':
      carlist.save_labels(labels_filename);
      carlist.save_labels_simple(simple_labels_filename,
          velodyne_index.num_spins);
      break;
    case '!':
      delta = -1;
      break;
    case '@':
      delta = -2;
      break;
    case '#':
      delta = -3;
      break;
    case '$':
      delta = -4;
      break;
    case '%':
      delta = -5;
      break;
    case '^':
      delta = -6;
      break;
    case '&':
      delta = -7;
      break;
    case '*':
      delta = -8;
      break;
    case '(':
      delta = -9;
      break;
    case 'h':
      hold_spin_num = current_spin_num;
      hold_spin.load(velodyne_file, velodyne_config, &velodyne_index, 
          hold_spin_num, &applanix_lat, &applanix_lon,
          &applanix_alt);
      break;
    case 'H':
      hold_spin_num = -1;
      break;
    case 'f':
      draw_flat = !draw_flat;
      break;
    case 'm': case 't':
      if(last_car_num != -1) {
        p = carlist.car[last_car_num].get_pose(current_spin_num);
        if(p == NULL)
          dgc_die("Error: got NULL from get_pose when I shouldn't\n");
        p->x = scene_x;
        p->y = scene_y;
      }
      break;
    case 'd':
      for(i = 0; i < carlist.num_cars(); i++)
        if(carlist.car[i].car_selected(scene_x, scene_y, current_spin_num)) {
          carlist.car[i].delete_pose(current_spin_num);
          break;
        }
      break;
    case 'c':
      carlist.add_car(scene_x, scene_y, last_car_theta, DGC_PASSAT_WIDTH * 1.2,
          DGC_PASSAT_LENGTH * 1.2, current_spin_num, current_time);
      last_car_num = carlist.num_cars() - 1;
      break;
    case 'l':
      if(last_car_num != -1) {
        p = carlist.car[last_car_num].get_pose(current_spin_num);
        if(p == NULL)
          dgc_die("got NULL from get_pose when I shouldn't\n");
        switch(p->signal) {
          case TURN_SIGNAL_UNKNOWN:
            p->signal = TURN_SIGNAL_NONE;
            break;
          case TURN_SIGNAL_NONE:
            p->signal = TURN_SIGNAL_LEFT;
            break;
          case TURN_SIGNAL_LEFT:
            p->signal = TURN_SIGNAL_RIGHT;
            break;
          case TURN_SIGNAL_RIGHT:
            p->signal = TURN_SIGNAL_UNKNOWN;
            break;
        }
      }
      break;
    case 'p':
      for(i = 0; i < carlist.num_cars(); i++)
        if(carlist.car[i].car_selected(scene_x, scene_y, current_spin_num)) {
          if(!carlist.car[i].fixed) {
            p = carlist.car[i].get_pose(current_spin_num);
            p2 = *p;
            carlist.car[i].pose.clear();
            p = carlist.car[i].get_pose(current_spin_num);
            *p = p2;
            carlist.car[i].start_cap = 0;
            carlist.car[i].end_cap = 0;
            carlist.car[i].fixed = 1;
          }
          else {
            carlist.car[i].fixed = 0;
            carlist.car[i].pose[0].t = current_spin_num;
            carlist.car[i].pose[0].timestamp = current_time;
            carlist.car[i].start_cap = 1;
            carlist.car[i].end_cap = 0;
          }
          last_car_num = i;
          break;
        }
      break;
    case ' ':
      for(i = 0; i < carlist.num_cars(); i++)
        if(carlist.car[i].car_selected(scene_x, scene_y, current_spin_num)) {
          p = carlist.car[i].get_pose(current_spin_num);
          last_car_num = i;
          break;
        }
      break;
    case 'z':
      for(i = 0; i < carlist.num_cars(); i++)
        if(carlist.car[i].car_selected(scene_x, scene_y, current_spin_num)) {
          if(carlist.car[i].num_poses() > 0) {
            n = carlist.car[i].num_poses();
            if(current_spin_num == carlist.car[i].pose[0].t &&
                carlist.car[i].start_cap)
              carlist.car[i].start_cap = 0;
            else if(current_spin_num <= carlist.car[i].pose[0].t) {
              p = carlist.car[i].get_pose(current_spin_num);
              carlist.car[i].start_cap = 1;
            }
            else if(current_spin_num == carlist.car[i].pose[n - 1].t &&
                carlist.car[i].end_cap)
              carlist.car[i].end_cap = 0;
            else if(current_spin_num >= carlist.car[i].pose[n - 1].t) {
              p = carlist.car[i].get_pose(current_spin_num);
              carlist.car[i].end_cap = 1;
            }
            last_car_num = i;
          }
          break;
        }
      break;
    default:
      break;
    }

  if(delta) {
    current_spin_num += delta;
    if(current_spin_num >= velodyne_index.num_spins)
      current_spin_num = velodyne_index.num_spins - 1;
    if(current_spin_num < 0)
      current_spin_num = 0;
    spin.load(velodyne_file, velodyne_config, &velodyne_index, current_spin_num,
        &applanix_lat, &applanix_lon, &applanix_alt);

    if (velodyne_index.spin[current_spin_num].num_poses > 0)
      current_time = velodyne_index.spin[current_spin_num].pose[velodyne_index.spin[current_spin_num].num_poses - 1].timestamp;
    else current_time = 0.0;
  }

  gui3D_forceRedraw();

}

void mouse(__attribute__ ((unused)) int button, 
    int state, int x, int y)
{
  double applanix_lat, applanix_lon, applanix_alt;
  double scene_x, scene_y;
  int i;

  /* figure out where the user clicked on the plane */
  gui3D_pick_point(x, y, &scene_x, &scene_y);
  scene_x += velodyne_index.spin[current_spin_num].pose[0].smooth_x;
  scene_y += velodyne_index.spin[current_spin_num].pose[0].smooth_y;

  if(gui3D.modifiers & GLUT_ACTIVE_CTRL) {
    if(state == GLUT_DOWN) {
      if(edit_mode == MODE_NONE) {
        for(i = 0; i < carlist.num_cars(); i++)
          if(carlist.car[i].center_selected(scene_x, scene_y, current_spin_num)) {
            edit_mode = MODE_SELECT_CENTER;
            edit_car_num = i;
            last_car_num = i;
            break;
          }
          else if(carlist.car[i].front_selected(scene_x, scene_y, current_spin_num)) {
            edit_mode = MODE_SELECT_FRONT;
            edit_car_num = i;
            last_car_num = i;
          }
          else if(carlist.car[i].corner_selected(scene_x, scene_y, current_spin_num)) {
            edit_mode = MODE_SELECT_CORNER;
            edit_car_num = i;
            last_car_num = i;
          }
      }
    }
    else if(state == GLUT_UP)  {
      if(edit_mode != MODE_NONE) {
        edit_mode = MODE_NONE;
        edit_car_num = -1;
      }
      spin.load(velodyne_file, velodyne_config, &velodyne_index, 
          current_spin_num, &applanix_lat, &applanix_lon, &applanix_alt);

      if (velodyne_index.spin[current_spin_num].num_poses > 0)
        current_time = velodyne_index.spin[current_spin_num].pose[velodyne_index.spin[current_spin_num].num_poses - 1].timestamp;
      else current_time = 0.0;
    }
  }
  last_mouse_x = x;
  last_mouse_y = y;
}

// Find the best lat, lon offset for the car given a fixed orientation. Also report a score for how good this alignment is

void best_offset_fixed_theta(float dtheta, float *px, float *py, float *pz, int car_points, float min_z, car_pose_t *p, car_t *c, float *best_offset_lat, float *best_offset_lon, float *score) {
  int hist_lat[80], hist_lon[80];
  int i;
  float x, y, z;
  for(i = 0; i < 80; i++)
    hist_lat[i] = hist_lon[i] = 0;
  float p_theta = wrap(p->theta + dtheta);
  float my_x = velodyne_index.spin[current_spin_num].pose[0].smooth_x;
  float my_y = velodyne_index.spin[current_spin_num].pose[0].smooth_y;
  float bearing = p_theta - atan2(my_y - p->y, my_x - p->x);
  // printf("Direction to Junior: %.2f\n", bearing * 180/M_PI);
  int lat_side = -1, lon_side = -1;
  // Relative weightings of lateral and longitudinal confidences depending on our orientation to this car
  float lat_confidence = .2 + fabs(sin(bearing));
  float lon_confidence = .1 + fabs(cos(bearing));
  // printf("Confidence --> Lat: %.2f   Lon: %.2f\n", lat_confidence, lon_confidence);
  if(bearing < 0) // Junior is to our left, so align the left edge
    lat_side = LEFT;
  else
    lat_side = RIGHT;
  if(fabs(bearing) > M_PI/2)
    lon_side = BACK;
  else
    lon_side = FRONT;
  for(i = 0; i < car_points; i++) { // put car points into lat, lon histograms
    x = px[i];
    y = py[i];
    z = pz[i];
    if(z < min_z + .3)
      continue;
    float r = hypot(x, y);
    float theta2 = atan2(y, x);
    float lat = -r * sin(theta2 - (p_theta + 0*M_PI/2));
    float lon = -r * sin(theta2 - (p_theta + M_PI/2));
    int hist_bin_lon = 10 * (lon + 4);
    int hist_bin_lat = 10 * (lat + 2);
    if(hist_bin_lat >= 0 && hist_bin_lat <= 40)
      hist_lat[hist_bin_lat]++;
    if(hist_bin_lon >= 0 && hist_bin_lon <= 80)
      hist_lon[hist_bin_lon]++;
  }

  int max_offset_lat = -1;
  int max_count_lat = -1;
  int min_count_lat = 100000;
  int start = 0, stop = 20;
  if(lat_side == RIGHT) {
    start = 20;
    stop = 40;
  }
  for(i = start; i < stop; i++) {
    if(hist_lat[i] > max_count_lat) {
      max_count_lat = hist_lat[i];
      max_offset_lat = i;
    }
    if(hist_lat[i] < min_count_lat)
      min_count_lat = hist_lat[i];
    //printf("LAT Offset %.2f --> %d points\n", offset, hist_lat[i]);
  }
  // printf("Max lat offset: %.2f\n", .1 * max_offset_lat - 2);
  int end = 0, dir = -1;
  if(lat_side == RIGHT) {
    end = 40;
    dir = 1;
  }
  for(i = max_offset_lat; i != end; i += dir) {
    if((hist_lat[i] - min_count_lat) < .15 * (max_count_lat - min_count_lat)) {
      break;
    }
  }
  if(hist_lat[i] - min_count_lat > .05 * max_count_lat)
    i += dir;
  // printf("Using offset: %.2f\n", .1 * i - 2);
  float correction_lat = (.1 * i - 2) - dir * (c->w)/2;
  // printf("Need to apply a lateral correction of %.2f\n", correction_lat);

  int max_offset_lon = -1;
  int max_count_lon = -1;
  int min_count_lon = 100000;
  start = 0; stop = 30;
  if(lon_side == FRONT) {
    start = 50;
    stop = 80;
  }
  for(i = start; i < stop; i++) {
    if(hist_lon[i] > max_count_lon) {
      max_count_lon = hist_lon[i];
      max_offset_lon = i;
    }
    if(hist_lon[i] < min_count_lon)
      min_count_lon = hist_lon[i];
    //printf("LON Offset %.2f --> %d points\n", offset, hist_lon[i]);
  }
  // printf("Max lon offset: %d = %.2fm\n", max_offset_lon, .1 * max_offset_lon - 4);
  end = 0; dir = -1;
  if(lon_side == FRONT) {
    end = 80;
    dir = 1;
  }
  for(i = max_offset_lon; i != end; i += dir) {
    if((hist_lon[i] - min_count_lon) < .2 * (max_count_lon - min_count_lon)) {
      break;
    }
  }
  i += dir;
  // printf("Using longitudinal offset: %.2f\n", .1 * i - 4);
  float correction_lon = (.1 * i - 4) - dir * (c->l)/2;
  // printf("Need to apply a longitudinal correction of %.2f\n", correction_lon);

  *best_offset_lat = correction_lat;
  *best_offset_lon = correction_lon;
  *score = 2.0 * max_count_lat * lat_confidence + 1.0 * max_count_lon * lon_confidence;
  // printf("SCORE for angle %.2f: %f\n", dtheta * 180 / M_PI, *score);
  return;
}

void car_list_t::rotate_car(float dtheta) {
  if(last_car_num < 0) {
    printf("  Sorry, couldn't find a car.\n");
  }
  car_pose_t *p;
  car_t *c;
  c = &carlist.car[last_car_num];
  p = c->get_pose(current_spin_num);
  p->theta = wrap(p->theta + dtheta);
  last_car_theta = p->theta;
}


int align_car(void) {
  car_pose_t *p;
  car_t *c;
  int i, j;
  double x, y, z, theta;
  dgc_velodyne_spin *vspin = &spin;
  // printf("Trying to align car %d\n", last_car_num);
  if(last_car_num < 0) {
    printf("  Sorry, couldn't find a car.\n");
    return -1;
  }
  // printf("   We have a car!\n");
  c = &carlist.car[last_car_num];
  p = c->get_pose(current_spin_num);
  printf("   Car at %f x %f\n", p->x, p->y);
  printf("   Width: %f   Length: %f\n", c->w, c->l);

  // First, we find Velodyne points near where the user clicked

  int car_points = 0, max_points = 5000;
  float px[max_points], py[max_points], pz[max_points]; // to store points near the car
  float min_z = 100000.0;
  for(i = 0; i < vspin->num_scans; i++) {
    for(j = 0; j < VELO_BEAMS_IN_SCAN && car_points < max_points; j++) {
      if(vspin->scans[i].p[j].range < 0.01)
        continue;
      x = vspin->scans[i].p[j].x * 0.01 + vspin->scans[i].robot.x;
      y = vspin->scans[i].p[j].y * 0.01 + vspin->scans[i].robot.y;
      z = vspin->scans[i].p[j].z * 0.01 + vspin->scans[i].robot.z;
      if(hypot(p->x - x, p->y - y) < 4 && car_points < max_points) {
        px[car_points] = x - p->x;
        py[car_points] = y - p->y;
        pz[car_points] = z;
        if(z < min_z) min_z = z;
        car_points++;
        //printf("Added point (%.2f, %.2f)\n", x - p->x, y - p->y);
      }
    }
  }
  // printf("  We saw %d points in the car!\n", car_points);
  if(car_points < 5) {
    printf("  Sorry, not enough points to align this car.\n");
    return -1;
  }
  theta = p->theta;

  float correction_lat, correction_lon, score;
  float dtheta;
  float best_correction_lat = 0, best_correction_lon = 0, best_theta = 0, best_score = -1;

  // Search for best theta, medium resolution

  for(dtheta = -M_PI/6 * 1; dtheta <= M_PI/6 * 1; dtheta += M_PI/48) {
    best_offset_fixed_theta(dtheta, px, py, pz, car_points, min_z, p, c, &correction_lat, &correction_lon, &score);
    if(score > best_score) {
      best_correction_lat = correction_lat;
      best_correction_lon = correction_lon;
      best_score = score;
      best_theta = dtheta;
    }
  }
  printf("===============\n");
  printf("Best dtheta: %.2f\n", best_theta * 180/M_PI);
  printf("Best lateral offset: %.2f\n", best_correction_lat);
  printf("Best longitudinal offset: %.2f\n", best_correction_lon);

  // Now do a high-resolution search near the best theta

  for(dtheta = best_theta - M_PI/48; dtheta <= best_theta + M_PI/48; dtheta += M_PI / 192) {
    best_offset_fixed_theta(dtheta, px, py, pz, car_points, min_z, p, c, &correction_lat, &correction_lon, &score);
    if(score > best_score) {
      best_correction_lat = correction_lat;
      best_correction_lon = correction_lon;
      best_score = score;
      best_theta = dtheta;
    }
  }
  printf("===============\n");
  printf("Best refined dtheta: %.2f\n", best_theta * 180/M_PI);
  printf("Best refined lateral offset: %.2f\n", best_correction_lat);
  printf("Best refined longitudinal offset: %.2f\n", best_correction_lon);
  p->theta = wrap(p->theta + best_theta);
  p->x -= best_correction_lat * cos(p->theta + M_PI/2);
  p->y -= best_correction_lat * sin(p->theta + M_PI/2);
  p->x += best_correction_lon * cos(p->theta);
  p->y += best_correction_lon * sin(p->theta);

  printf("EXITING ALIGNMENT\n");
  return 0;
}

void motion(int x, int y)
{
  double scene_x, scene_y;
  car_pose_t *p = NULL;

  /* figure out where the user clicked on the plane */
  gui3D_pick_point(x, y, &scene_x, &scene_y);
  scene_x += velodyne_index.spin[current_spin_num].pose[0].smooth_x;
  scene_y += velodyne_index.spin[current_spin_num].pose[0].smooth_y;

  if(gui3D.modifiers & GLUT_ACTIVE_CTRL)  {
    if(edit_mode == MODE_NONE) {
      /*      current_spin_num += (x - last_mouse_x);
      if(current_spin_num >= velodyne_index.num_spins)
	current_spin_num = velodyne_index.num_spins - 1;
      if(current_spin_num < 0)
      current_spin_num = 0;*/
      gui3D_forceRedraw();
    }
    else if(edit_mode == MODE_SELECT_CENTER) {
      p = carlist.car[edit_car_num].get_pose(current_spin_num);
      if(p == NULL)
        dgc_die("Error: got NULL from get_pose when I shouldn't\n");
      p->x = scene_x;
      p->y = scene_y;
    }
    else if(edit_mode == MODE_SELECT_FRONT) {
      p = carlist.car[edit_car_num].get_pose(current_spin_num);
      if(p == NULL)
        dgc_die("Error: got NULL from get_pose when I shouldn't\n");

      //      carlist.car[edit_car_num].l = 2 * hypot(scene_x - p->x, scene_y - p->y);
      p->theta = atan2(scene_y - p->y, scene_x - p->x);
      last_car_theta = p->theta;
    }
    else if(edit_mode == MODE_SELECT_CORNER) {
      p = carlist.car[edit_car_num].get_pose(current_spin_num);
      if(p == NULL)
        dgc_die("Error: got NULL from get_pose when I shouldn't\n");
      carlist.car[edit_car_num].l = 
          fabs(2 * ((scene_x - p->x) * cos(p->theta) +
              (scene_y - p->y) * sin(p->theta)));

      carlist.car[edit_car_num].w = 
          fabs(2 * ((scene_x - p->x) * -sin(p->theta) +
              (scene_y - p->y) * cos(p->theta)));
    }
    if (p != NULL)
      p->timestamp = current_time;
  }
  last_mouse_x = x;
  last_mouse_y = y;
}

void draw_spin(dgc_velodyne_spin *vspin, double origin_x, double origin_y,
    double origin_z, dgc_transform_t t, int color_mode, int flat)
{
  int i, j;
  double x, y, z, u;

  if(flat)
    glDisable(GL_DEPTH_TEST);

  glBegin(GL_POINTS);
  for(i = 0; i < vspin->num_scans; i++) 
    for(j = 0; j < VELO_BEAMS_IN_SCAN; j++) {
      if(vspin->scans[i].p[j].range < 0.01)
        continue;

      x = vspin->scans[i].p[j].x * 0.01 + vspin->scans[i].robot.x;
      y = vspin->scans[i].p[j].y * 0.01 + vspin->scans[i].robot.y;
      z = vspin->scans[i].p[j].z * 0.01 + vspin->scans[i].robot.z;
      dgc_transform_point(&x, &y, &z, t);

      if(color_mode == 1) {
        u = 0.5 + (vspin->scans[i].p[j].intensity - 40.0)/80.0;
        if(u < 0) u = 0;
        if(u > 1) u = 1;
        glColor3f(u, u, u);
      }
      else if(color_mode == 2) {
        u = (0.01 * vspin->scans[i].p[j].z +
            DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT) / 3.0;
        if(u > 1)
          u = 1;
        else if(u < 0)
          u = 0;
        glColor3f(1 - u, u, 0);
      }

      if(flat)
        glVertex3f(x - origin_x, y - origin_y, 0);
      else {
        if(z - origin_z < 0.05)
          glVertex3f(x - origin_x, y - origin_y, 0.05);
        else
          glVertex3f(x - origin_x, y - origin_y, z - origin_z);
      }

    }
  glEnd();

  if(flat)
    glEnable(GL_DEPTH_TEST);
}

void draw_path(double origin_x, double origin_y)
{
  double dx, dy;
  int i;

  /* draw path before and after current scan */
  glDisable(GL_DEPTH_TEST);

  glColor4f(1, 1, 0, 0.3);
  glBegin(GL_QUAD_STRIP);
  for(i = 0; i <= current_spin_num; i++) {
    dx = cos(velodyne_index.spin[i].pose[0].yaw + M_PI / 2.0);
    dy = sin(velodyne_index.spin[i].pose[0].yaw + M_PI / 2.0);
    glVertex2f(velodyne_index.spin[i].pose[0].smooth_x + dx - origin_x,
        velodyne_index.spin[i].pose[0].smooth_y + dy - origin_y);
    glVertex2f(velodyne_index.spin[i].pose[0].smooth_x - dx - origin_x,
        velodyne_index.spin[i].pose[0].smooth_y - dy - origin_y);
  }
  glEnd();

  glColor4f(0, 0, 1, 0.3);
  glBegin(GL_QUAD_STRIP);
  for(i = current_spin_num; i < velodyne_index.num_spins; i++) {
    dx = cos(velodyne_index.spin[i].pose[0].yaw + M_PI / 2.0);
    dy = sin(velodyne_index.spin[i].pose[0].yaw + M_PI / 2.0);
    glVertex2f(velodyne_index.spin[i].pose[0].smooth_x + dx - origin_x,
        velodyne_index.spin[i].pose[0].smooth_y + dy - origin_y);
    glVertex2f(velodyne_index.spin[i].pose[0].smooth_x - dx - origin_x,
        velodyne_index.spin[i].pose[0].smooth_y - dy - origin_y);
  }
  glEnd();

  glEnable(GL_DEPTH_TEST);
}

void draw_info_box(void)
{
  char str[200];
  double u;

  glLineWidth(3);
  glColor4f(0, 0, 0, 0.5);
  glBegin(GL_POLYGON);
  glVertex2f(0, 0);
  glVertex2f(gui3D.window_width, 0);
  glVertex2f(gui3D.window_width, 50);
  glVertex2f(0, 50);
  glEnd();
  glColor3f(1, 1, 1);
  glBegin(GL_LINE_LOOP);
  glVertex2f(0, 0);
  glVertex2f(gui3D.window_width, 0);
  glVertex2f(gui3D.window_width, 50);
  glVertex2f(0, 50);
  glEnd();

  glBegin(GL_LINES);
  glVertex2f(20, 25);
  glVertex2f(gui3D.window_width - 20, 25);
  u = current_spin_num / (double)velodyne_index.num_spins * 
      (gui3D.window_width - 40.0);
  glVertex2f(20 + u, 10);
  glVertex2f(20 + u, 40);
  glEnd();

  glColor3f(1, 1, 0);
  sprintf(str, "%d of %d", current_spin_num, velodyne_index.num_spins);
  renderBitmapString(gui3D.window_width - 20 - 
      bitmapStringWidth(GLUT_BITMAP_HELVETICA_18, str),
      31, GLUT_BITMAP_HELVETICA_18, str);
}

void display(void)
{
  double robot_lat, robot_lon, robot_x, robot_y, robot_z, robot_roll;
  double robot_pitch, robot_yaw, robot_smooth_x, robot_smooth_y;
  double robot_smooth_z;
  dgc_transform_t t;
  char utmzone[10];
  int i;

  /*  fprintf(stderr, "%d cars\n", carlist.num_cars());
  for(i = 0; i < carlist.num_cars(); i++) {
    fprintf(stderr, "  %d : %d : ", i, carlist.car[i].num_poses());
    for(j = 0; j < carlist.car[i].num_poses(); j++)
      fprintf(stderr, "%d ", carlist.car[i].pose[j].t);
    fprintf(stderr, "\n");
    }*/


  /* clear to black */
  glClearColor(1, 1, 1, 0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  /* turn on snooth lines */
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_LINE_SMOOTH);

  /* calculate origin */
  robot_lat = velodyne_index.spin[current_spin_num].pose[0].latitude;
  robot_lon = velodyne_index.spin[current_spin_num].pose[0].longitude;
  latLongToUtm(robot_lat, robot_lon, &robot_x, &robot_y, utmzone);
  robot_z = velodyne_index.spin[current_spin_num].pose[0].altitude;
  robot_smooth_x = velodyne_index.spin[current_spin_num].pose[0].smooth_x;
  robot_smooth_y = velodyne_index.spin[current_spin_num].pose[0].smooth_y;
  robot_smooth_z = velodyne_index.spin[current_spin_num].pose[0].smooth_z;
  robot_roll = velodyne_index.spin[current_spin_num].pose[0].roll;
  robot_pitch = velodyne_index.spin[current_spin_num].pose[0].pitch;
  robot_yaw = velodyne_index.spin[current_spin_num].pose[0].yaw;

  /* draw aerial imagery */
  glPushMatrix();
  glTranslatef(0, 0, 0.2);
  dgc_imagery_draw_3D(imagery_root, gui3D.camera_pose.distance,
      gui3D.camera_pose.x_offset,
      gui3D.camera_pose.y_offset,
      robot_x, robot_y, utmzone, true, 1.0, 1);
  glPopMatrix();

  /* draw robot path */
  draw_path(robot_smooth_x, robot_smooth_y);

  glDisable(GL_DEPTH_TEST);

  for(i = 0; i < carlist.num_cars(); i++) 
    draw_car(&carlist.car[i], robot_smooth_x, robot_smooth_y, current_spin_num);

#ifdef blah

  /* draw current cars */
  for(i = 0; i < carlist.num_cars(); i++) {
    glColor3f(1, 1, 0);
    carlist[current_spin_num].car[i].draw(robot_smooth_x, robot_smooth_y);
  }

  for(i = current_spin_num - 1; i >= 0; i--) 
    for(j = 0; j < carlist[i].num_cars(); j++) {
      id = carlist[i].car[j].id;
      new_id = 1;
      for(k = 0; k < (int)used_ids.size(); k++) 
        if(id == used_ids[k]) {
          new_id = 0;
          break;
        }
      if(new_id) {
        if(!carlist[i].car[j].end_cap) {
          glColor4f(1, 1, 0, 0.5);
          carlist[i].car[j].draw(robot_smooth_x, robot_smooth_y);
        }
        used_ids.push_back(id);
      }
    }
#endif

  glEnable(GL_DEPTH_TEST);

  /* draw the velodyne spins */
  glColor3f(1, 1, 1);
  if(large_points)
    glPointSize(3);
  else
    glPointSize(1);
  dgc_transform_identity(t);
  if(hold_spin_num != -1) 
    draw_spin(&hold_spin, robot_smooth_x, robot_smooth_y, robot_smooth_z -
        DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT, t, color_mode, draw_flat);
  draw_spin(&spin, robot_smooth_x, robot_smooth_y, robot_smooth_z -
      DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT, t, color_mode, draw_flat);

  /* draw the passat */
  glEnable(GL_LIGHTING);
  glPushMatrix();
  glRotatef(dgc_r2d(robot_yaw), 0, 0, 1);
  glRotatef(dgc_r2d(robot_pitch), 0, 1, 0);
  glRotatef(dgc_r2d(robot_roll), 1, 0, 0);
  glTranslatef(1.65, 0, -0.6 + DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT);
  passatwagonmodel_draw(passat, 0, 0, 0);
  glPopMatrix();
  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND);

  /* go to 2D */
  set_display_mode_2D(gui3D.window_width, gui3D.window_height);

  /* draw the info box */
  draw_info_box();
}

void timer(int)
{
  if(dgc_imagery_update())
    gui3D_forceRedraw();
  gui3D_add_timerFunc(100, timer, 0);
}

void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = {
      {"transform", "velodyne", DGC_PARAM_TRANSFORM,   &velodyne_offset, 1, NULL},
      {"velodyne", "cal_file", DGC_PARAM_FILENAME, &cal_filename, 0, NULL},
      {"imagery", "root", DGC_PARAM_FILENAME, &imagery_root, 0, NULL},
  };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
}

int main(int argc, char **argv)
{
  char vlf_filename[300], index_filename[300];
  double applanix_lat, applanix_lon, applanix_alt;
  IpcInterface *ipc;
  ParamInterface *pint;

  if(argc < 2)
    dgc_die("Error: not enough arguments.\n"
        "Usage: %s vlf-file\n", argv[0]);

  /* IPC initialization */
  ipc = new IpcStandardInterface();
  pint = new ParamInterface(ipc);
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  read_parameters(pint, argc, argv);
  delete ipc;

  strcpy(vlf_filename, argv[1]);
  if(strlen(vlf_filename) < 4 || 
      strcmp(vlf_filename + strlen(vlf_filename) - 4, ".vlf"))
    dgc_die("Error: first argument must end in .vlf\n");

  if(argc >= 3)
    strcpy(index_filename, argv[2]);
  else {
    strcpy(index_filename, argv[1]);
    strcat(index_filename, ".index.gz");
  }

  strcpy(labels_filename, vlf_filename);
  strcat(labels_filename, ".labels");

  strcpy(simple_labels_filename, vlf_filename);
  strcat(simple_labels_filename, ".labels.txt");

  carlist.load_labels(labels_filename);

  /* open velodyne file */
  velodyne_file = dgc_velodyne_open_file(vlf_filename);
  if(velodyne_file == NULL)
    dgc_die("Error: Could not open velodyne file %s for reading.\n", 
        vlf_filename);

  /* load the velodyne index */

  if (velodyne_index.load(index_filename) < 0) {
    dgc_die("Error: Could not load velodyne index %s\n", index_filename);
  }

  /* load velodyne calibration & transform */
  dgc_velodyne_get_config(&velodyne_config);
  if(dgc_velodyne_read_calibration(cal_filename, velodyne_config) != 0) {
    fprintf(stderr, "# ERROR: could not read calibration file!\n");
    exit(0);
  }
  dgc_velodyne_integrate_offset(velodyne_offset, velodyne_config);

  spin.load(velodyne_file, velodyne_config, &velodyne_index, current_spin_num,
      &applanix_lat, &applanix_lon, &applanix_alt);

  current_time = velodyne_index.spin[current_spin_num].pose[0].timestamp;

  if (velodyne_index.spin[current_spin_num].num_poses > 0)
    current_time = velodyne_index.spin[current_spin_num].pose[velodyne_index.spin[current_spin_num].num_poses - 1].timestamp;
  else current_time = 0.0;

  /* setup GUI */
  gui3D_initialize(argc, argv, 10, 10, 720, 480, 10.0);
  gui3D_setCameraParams(0.2 * .05, 0.5, 0.001, 10.5, 30, 1, 60000);
  gui3D_set_displayFunc(display);
  gui3D_set_motionFunc(motion);
  gui3D_set_mouseFunc(mouse);
  gui3D_set_keyboardFunc(keyboard);
  gui3D_add_timerFunc(100, timer, 0);
  passat = passatwagonmodel_load(0.0, 0.0, 0.5, 1);
  dgc_imagery_set_imagery_type(DGC_IMAGERY_TYPE_COLOR);

  gui3D_mainloop();
  return 0;
}
