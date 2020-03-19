#include <roadrunner.h>
#include <grid.h>
#include <lltransform.h>
#include <velodyne_interface.h>
#include <applanix_interface.h>
#include <param_interface.h>
#include <velodyne_interface.h>
#include <localize_interface.h>
#include <gls_interface.h>
#include <velocore.h>
#include <velo_support.h>
#include <ipc_std_interface.h>

#include "veloclient.h"
#include "terrainmap.h"

#define UTM_ZONE "10S"

// filter parameters
#define GPS_ERR 1.5
#define SENSOR_WEIGHT 0.005
#define MEASUREMENT_WEIGHT 0.1
#define MOTION_NOISE 0.1

#define SHOW_GLS 1
#define GRID_RADIUS 16

using namespace dgc;
using namespace vlr;

void dgc_localize_publish_pose_message(IpcInterface *ipc, LocalizePose *pose);

#define DIM 500
#define RES .15
#define DEBUG 0
#define PUBLISH_INTERVAL 0.1
#define LOCALIZE_DISTANCE_INTERVAL 0.2
#define LOCALIZE_TIME_INTERVAL 0.2
#define SMOOTHNESS 0.9

FILE *outlog = NULL;

dgc::IpcInterface              *ipc = NULL;


GlsOverlay *gls;

/* grid stuff */

typedef struct {
  double intensity_sum;
  int intensity_count;
  int n;
  float xa;
  float sxi2;
  float stdev;
} map_cell_t, *map_cell_p;

dgc_grid_p grid;
int grid_updated = 0;

typedef struct {
  terrain_tile *tile;
} grid_cell_t;

double hist_z0[GRID_RADIUS * 3][GRID_RADIUS * 3];
double hist_z[GRID_RADIUS * 3][GRID_RADIUS * 3];
double posterior[GRID_RADIUS * 3][GRID_RADIUS * 3];
double posterior1[GRID_RADIUS * 3][GRID_RADIUS * 3];
int has_localized = 0;
static double last_x_offset = 0, last_y_offset = 0;

double timestamp = 0;
double localize_x_offset = 0, localize_y_offset = 0;
double utm_x = 0, utm_y = 0, smooth_x = 0, smooth_y = 0, smooth_z = 0, yaw = 0;

dgc_grid_p tile_grid = NULL;

float r[101], g[101], b[101];

/* IPC */

static pthread_mutex_t posterior_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t velodyne_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t applanix_pose_mutex = PTHREAD_MUTEX_INITIALIZER;
int received_applanix_pose = 0;
ApplanixPose applanix_pose;

/* parameters */

char *imagery_root;
char *int_filename;
char *cal_filename;
int  calibrate_intensities;
dgc_transform_t velodyne_offset;

//map_cell_t sensor_data[DIM * DIM];
map_cell_t *sensor_data;
terrain_tile_cell **map_data = NULL;


void init_colors() {
	int i;
	for(i = 0; i < 50; i++) {
		r[i] = .5 + .01 * i;
		g[i] = .5 + .01 * i;
		b[i] = .5 - .01 * i;
	}
	for(i = 50; i <= 100; i++) {
		r[i] = 1;
		g[i] = 2 - .02 * i;
		b[i] = 0;
	}
}

void my_glsSend(GlsOverlay *gls) {
  //printf("Sending GLS Size: %d bytes\n", gls->num_bytes);
  glsSend(ipc, gls);
}

void 
tile_clear_handler(void *cell) 
{
  terrain_tile *tile = ((grid_cell_t *)cell)->tile;
  if(tile != NULL) {
    delete ((grid_cell_t *)cell)->tile;
  }
}

float gaussian_weights[100][40];

double prob_normal(double a, double b) { // probability of a given stdev b
  //float p = pow(2 * 3.14159 * b * b, -.5) * exp(-.5 * a * a / (b * b));
  float p = exp(-.5 * a * a / (b * b));
  //if(p < .001) return .001;
  return p;
}

void init_guassians() {
	int i, j;
	for(i = 0; i < 100; i++) {
		for(j = 0; j < 40; j++) {
			float p = 2 * prob_normal(1.0 * i, 1.0 * j);
			if(p < .02) p = .02;
			gaussian_weights[i][j] = log(p);
			//printf("%d %d: %.6f %.6f\n", i, j, prob_normal(1.0*i,1.0*j), gaussian_weights[i][j]);
		}
	}
}

void plot_2D(void) {
	//printf("plotting 2D histogram\n");
	gls_clear(gls);
	gls->coordinates = GLS_LOCAL_COORDINATES;
	glsColor3f(gls, 1, 1, 0);
	glsLineWidth(gls, 2.0);
	double max_z = 0, sum_z = 0;
	int xpos, ypos;
	for(xpos = 0; xpos < GRID_RADIUS; xpos++) {
		for(ypos = 0; ypos < GRID_RADIUS; ypos++) {
			if(hist_z[xpos][ypos] > 0)
				sum_z += hist_z[xpos][ypos];
			if(hist_z[xpos][ypos] > max_z)
				max_z = hist_z[xpos][ypos];
		}
	}
		
	//max_z = sum_z / 50;
        double dx0 = -1 * (last_x_offset - utm_x + smooth_x);
        double dy0 = -1 * (last_y_offset - utm_y + smooth_y);
	for(xpos = 0; xpos < GRID_RADIUS; xpos++) {
		glsBegin(gls, GLS_LINE_STRIP);
		for(ypos = 0; ypos < GRID_RADIUS; ypos++) {
			if((xpos-GRID_RADIUS/2)*(xpos-GRID_RADIUS/2)+(ypos-GRID_RADIUS/2)*(ypos-GRID_RADIUS/2) > GRID_RADIUS*GRID_RADIUS/4)
				continue;
			float dx = dx0 + (2 * xpos - GRID_RADIUS) * RES;
			float dy = dy0 + (2 * ypos - GRID_RADIUS) * RES;
			double z = hist_z[xpos][ypos] / max_z;
      			double d = pow(dx * dx + dy * dy, .5);
      			double theta = M_PI/2 + atan2(dy, dx) - yaw;
      			double dlat = d * cos(theta);
      			double dlon = d * sin(theta);
			int c = (int)(100 * z);
			glsColor3f(gls, r[c], g[c], b[c]);
			glsVertex3f(gls, dlon, dlat, 2*z - 1.4);
		}
		glsEnd(gls);
	}
	for(ypos = 0; ypos < GRID_RADIUS; ypos++) {
		glsBegin(gls, GLS_LINE_STRIP);
		for(xpos = 0; xpos < GRID_RADIUS; xpos++) {
			if((xpos-GRID_RADIUS/2)*(xpos-GRID_RADIUS/2)+(ypos-GRID_RADIUS/2)*(ypos-GRID_RADIUS/2) > GRID_RADIUS*GRID_RADIUS/4)
				continue;
			float dx = dx0 + (2 * xpos - GRID_RADIUS) * RES;
			float dy = dy0 + (2 * ypos - GRID_RADIUS) * RES;
			double z = hist_z[xpos][ypos] / max_z;
      			double d = pow(dx * dx + dy * dy, .5);
      			double theta = M_PI/2 + atan2(dy, dx) - yaw;
      			double dlat = d * cos(theta);
      			double dlon = d * sin(theta);
			int c = (int)(100 * z);
			glsColor3f(gls, r[c], g[c], b[c]);
			glsVertex3f(gls, dlon, dlat, 2 * z - 1.4);
		}
		glsEnd(gls);
	}
	for(xpos = -GRID_RADIUS; xpos < GRID_RADIUS; xpos++) {
		glsBegin(gls, GLS_LINE_STRIP);
		for(ypos = 0; ypos < GRID_RADIUS; ypos++) {
			int x = xpos + ypos;
			int y = ypos;
			if(x < 0 || x >= GRID_RADIUS)
				continue;
			if((x-GRID_RADIUS/2)*(x-GRID_RADIUS/2)+(y-GRID_RADIUS/2)*(y-GRID_RADIUS/2) > GRID_RADIUS*GRID_RADIUS/4)
				continue;
			float dx = dx0 + (2 * x - GRID_RADIUS) * RES;
			float dy = dy0 + (2 * y - GRID_RADIUS) * RES;
			double z = hist_z[x][y] / max_z;
      			double d = pow(dx * dx + dy * dy, .5);
      			double theta = M_PI/2 + atan2(dy, dx) - yaw;
      			double dlat = d * cos(theta);
      			double dlon = d * sin(theta);
			int c = (int)(100 * z);
			glsColor3f(gls, r[c], g[c], b[c]);
			glsVertex3f(gls, dlon, dlat, 2 * z - 1.4);
		}
		glsEnd(gls);
	}
	for(xpos = 0; xpos < 2 * GRID_RADIUS; xpos++) {
		glsBegin(gls, GLS_LINE_STRIP);
		for(ypos = 0; ypos < GRID_RADIUS; ypos++) {
			int x = xpos - ypos;
			int y = ypos;
			if(x < 0 || x >= GRID_RADIUS)
				continue;
			if((x-GRID_RADIUS/2)*(x-GRID_RADIUS/2)+(y-GRID_RADIUS/2)*(y-GRID_RADIUS/2) > GRID_RADIUS*GRID_RADIUS/4)
				continue;
			float dx = dx0 + (2 * x - GRID_RADIUS) * RES;
			float dy = dy0 + (2 * y - GRID_RADIUS) * RES;
			double z = hist_z[x][y] / max_z;
      			double d = pow(dx * dx + dy * dy, .5);
      			double theta = M_PI/2 + atan2(dy, dx) - yaw;
      			double dlat = d * cos(theta);
      			double dlon = d * sin(theta);
			int c = (int)(100 * z);
			glsColor3f(gls, r[c], g[c], b[c]);
			glsVertex3f(gls, dlon, dlat, 2 * z - 1.4);
		}
		glsEnd(gls);
	}
        /* glsColor3f(gls, 1, 0, 1);
        glsLineWidth(gls, 8.0);
        glsBegin(gls, GLS_LINE_STRIP);
        glsVertex3f(gls, 0, 0, -2.0);
        glsVertex3f(gls, 0, 0, .5);
        glsEnd(gls); */
	my_glsSend(gls);
	return;
}

void normalize_posterior() { // mutex should be locked when this is called
	double sum = 0.00000000000000000000000000000000001;
	int i, j;
	for(i = 0; i <= GRID_RADIUS * 2; i++) {
		for(j = 0; j <= GRID_RADIUS * 2; j++) {
			//posterior[i][j] += .000000000001 / (GRID_RADIUS*GRID_RADIUS);
			sum += posterior[i][j];
		}
	}
        //printf("Sum: %10f\n", sum);
	for(i = 0; i <= GRID_RADIUS * 2; i++) 
		for(j = 0; j <= GRID_RADIUS * 2; j++)
			posterior[i][j] /= sum;
	for(i = 0; i <= GRID_RADIUS * 2; i++) {
		for(j = 0; j <= GRID_RADIUS * 2; j++) {
			//printf("%.6f ", posterior[i][j]);
		}
		//printf("\n");
	}
	//printf("\n");
}

void motion_model(double speed) {
  //printf("motion model! Traveling at %f m/s\n", speed);
  pthread_mutex_lock(&posterior_mutex);

  int i, j, x, y;
  // motion model stuff here
	for(i = 0; i <= GRID_RADIUS * 2; i++) {
		for(j = 0; j <= GRID_RADIUS * 2; j++) {
			posterior1[i][j] = 0.0;
			if((i-GRID_RADIUS)*(i-GRID_RADIUS)+(j-GRID_RADIUS)*(j-GRID_RADIUS) > GRID_RADIUS*GRID_RADIUS)
				continue;
			double weight_sum = 0.0;
			for(x = -2; x <= 2; x++) {
				for(y = -2; y <= 2; y++) {
					int col = j+y;
					int row = i+x;
					if((col-GRID_RADIUS)*(col-GRID_RADIUS)+(row-GRID_RADIUS)*(row-GRID_RADIUS) > GRID_RADIUS*GRID_RADIUS)
						continue;
					if(col < 0 || col > 2*GRID_RADIUS || row < 0 || row > 2*GRID_RADIUS)
						continue;
					double dist = pow(x*x+y*y, .5);
					double weight = prob_normal(dist, MOTION_NOISE*(speed+1));
					posterior1[i][j] += weight * posterior[row][col];
					weight_sum += weight;
				}
			}
			posterior1[i][j] /= weight_sum;
		}	
	}
	for(i = 0; i <= GRID_RADIUS * 2; i++)
		for(j = 0; j <= GRID_RADIUS * 2; j++)
			posterior[i][j] = posterior1[i][j];	
  
  normalize_posterior(); 

  pthread_mutex_unlock(&posterior_mutex);
}

void posterior_init() {
	printf("initializing posterior\n");
	int i, j;
	for(i = 0; i < GRID_RADIUS * 3; i++) {
		for(j = 0; j < GRID_RADIUS * 3; j++) {
			posterior[i][j] = 0;
			if((i-GRID_RADIUS)*(i-GRID_RADIUS)+(j-GRID_RADIUS)*(j-GRID_RADIUS) <= GRID_RADIUS*GRID_RADIUS)
				posterior[i][j] = 1.0;
		}
	}
	//posterior[10][10] = 2000.0;
	normalize_posterior();
}

void precompute_pointers() {
  //printf("Precomputing pointers...\n");
  if(!map_data)
    map_data = (terrain_tile_cell **) calloc(DIM * DIM, sizeof(void *));
  int row, col;
  map_cell_p cell;
  pthread_mutex_lock(&velodyne_mutex);
  for(row = DIM/8 - GRID_RADIUS; row < 7*DIM/8 + GRID_RADIUS; row++) {
    for(col = DIM/8 - GRID_RADIUS; col < 7*DIM/8 + GRID_RADIUS; col++) {
      cell = (map_cell_p) dgc_grid_get_rc_local(grid, row, col); // sensor data cell
      
      double x, y;
      dgc_grid_cell_to_xy(grid, cell, &x, &y);
      x = utm_x + x - smooth_x;
      y = utm_y + y - smooth_y;
      
      int r2, c2;
      r2 = (int)floor(y / tile_grid->resolution);
      c2 = (int)floor(x / tile_grid->resolution);
      grid_cell_t *tile_cell = (grid_cell_t *)dgc_grid_get_rc_global(tile_grid, r2, c2);
      if(tile_cell == NULL) {
	printf("ERROR: null tile!?\n");
	printf("row = %d, col = %d   x = %f, y = %f\n", row, col, x - utm_x, y - utm_y);
	continue;
      }
      if(tile_cell->tile == NULL) {    // need to initialize it
	tile_cell->tile = new terrain_tile(TERRAIN_TILE_SIZE, TERRAIN_TILE_SIZE);
  // @TODO: Get actual directory the graphic laser imagery is in for this
	sprintf(tile_cell->tile->filename, "%s/%s/lmap-%s-%d-%d-%06d-%06d.tf.gz",
    imagery_root, "lasermap", UTM_ZONE, (int)rint(TERRAIN_TILE_RESOLUTION * 100),
		TERRAIN_TILE_SIZE, c2, r2);
	fprintf(stderr, "Loading %s\n", tile_cell->tile->filename);
	if(tile_cell->tile->load(tile_cell->tile->filename) < 0) {
	  printf("... nothing here\n");
	  tile_cell->tile->utm_x0 = c2 * tile_grid->resolution;
	  tile_cell->tile->utm_y0 = r2 * tile_grid->resolution;
	  tile_cell->tile->resolution = TERRAIN_TILE_RESOLUTION;
	  strcpy(tile_cell->tile->utmzone, UTM_ZONE);
	}
      }
      int tile_x = (int)floor((x - tile_cell->tile->utm_x0) / TERRAIN_TILE_RESOLUTION);
      int tile_y = (int)floor((y - tile_cell->tile->utm_y0) / TERRAIN_TILE_RESOLUTION);
      terrain_tile_cell *grid_cell = &(tile_cell->tile->cell[tile_x][tile_y]);
      
      sensor_data[DIM * row + col] = *cell;
      map_data[DIM * row + col] = grid_cell;
    }
  }
  pthread_mutex_unlock(&velodyne_mutex);
  //printf("  .... done\n");
}

float inline correlation(float *x, float *y, int n) {
        if(n < 4)
                return 0.0;
        int i;
        double xsum = 0, ysum = 0;
        for(i = 0; i < n; i++) {
                xsum += x[i];
                ysum += y[i];
        }
        double xavg = xsum / n;
        double yavg = ysum / n;
        double xvar = 0, yvar = 0, cor = 0;
        for(i = 0; i < n; i++) {
                double dx = x[i] - xavg;
                double dy = y[i] - yavg;
                xvar += dx * dx;
                yvar += dy * dy;
                cor += dx * dy;
        }
        double xstd = pow(xvar / n, .5);
        double ystd = pow(yvar / n, .5);
        if(xstd == 0 || ystd == 0) {
                return 0;
        }
        double ret = cor / ((n - 1) * xstd * ystd);
        return ret;
}

float diff_array[DIM*DIM];
float stdev_array[DIM*DIM];

/* compute alignment strength between map tiles and current grid with cell offset dx, dy */
inline double align_strength(int dx, int dy)  { // turbo means skip every other pixel in both directions
  map_cell_t cell;
  int row, col, a, b, j;
  double log_sum = 0.0;
  int xrand = rand() % 2;
  int yrand = rand() % 2;
  float map_sum = 0.0, sens_sum = 0.0;
  int align_count = 0;
  for(row = yrand + DIM/6; row < 5*DIM/6; row += 2) {
    for(col = xrand + DIM/6; col < 5*DIM/6; col += 2) {
      cell = sensor_data[DIM * row + col];
      terrain_tile_cell *grid_cell = map_data[DIM * (row + dy) + col + dx];
      if(cell.intensity_count * grid_cell->i_count == 0) {
	continue;
      } else {	
        float intensity_map = grid_cell->intensity / grid_cell->i_count;
        float intensity_current = cell.intensity_sum / cell.intensity_count;
	map_sum += intensity_map;
	sens_sum += intensity_current;
	stdev_array[align_count] = (int) (grid_cell->stdev + cell.stdev);
	diff_array[align_count++] = intensity_current - intensity_map;
      }
    }
  }

  float map_average = map_sum / (align_count + 1);
  float sens_average = sens_sum / (align_count + 1);
  //printf("Map average: %.2f   Sensor average: %.2f\n", map_average, sens_average);
  float to_add = sens_average - map_average;
  float align_intensity_sum = 0.0;
  for(j = 0; j < align_count; j++) {
        align_intensity_sum += fabs(to_add - diff_array[j]);
	a = abs((int) (to_add - diff_array[j]));
        if(a > 99) a = 99;
	b = stdev_array[j];
        if(b < 5) b = 5;
        if(b > 39) b = 39;
        log_sum += SENSOR_WEIGHT * gaussian_weights[a][b];
  }

  //printf("old sum: %.6f  ", log_sum);
  //log_sum /= SENSOR_WEIGHT;
  //log_sum /= (align_count + 1);
  //log_sum *= 50;
  //printf("  new sum: %.6f matches: %d\n", log_sum, align_count);
  //float corr = correlation(map_array, sens_array, align_intensity_count) + .2;
  double gps_score = prob_normal(hypot(dx*RES, dy*RES), GPS_ERR);
  double gauss_score = exp(log_sum);
  //printf("    score: %10f\n", smart_score * gps_score);
  return(gauss_score * gps_score);
}

void compute_best_offset(void) {
  //printf("Localizing!!!\n");
  double t0, t1, t2, t3;
  t0 = dgc_get_time();
  precompute_pointers();
  t1 = dgc_get_time();
  if(DEBUG) printf("   t1 = %f\n", t1 - t0);
  int delta = 2, radius = GRID_RADIUS; // units = grid cells
  int offset_x = 0, offset_y = 0;
  int best_x = -1000, best_y = -1000;
  double best_strength = -1000;
  double measurement_model[GRID_RADIUS*3][GRID_RADIUS*3];
  int i, j;
  for(i = 0; i <= GRID_RADIUS * 2; i++) {
	for(j = 0; j <= GRID_RADIUS * 2; j++) {
		measurement_model[i][j] = 0;
	}
  }
  int xpos = 0, ypos = 0;
  for(offset_x = -radius + 1; offset_x < radius; offset_x += delta, xpos++) {
    ypos = 0;
    for(offset_y = -radius + 1; offset_y < radius; offset_y += delta, ypos++) {
      double strength = 0.0;
      if(offset_x * offset_x + offset_y * offset_y < (radius-2) * (radius-2)) {
	strength = align_strength(offset_x, offset_y);
      }
      hist_z0[xpos][ypos] = pow(strength, .05);

      measurement_model[2*ypos][2*xpos] = strength;
      measurement_model[2*ypos][2*xpos+1] = strength;
      measurement_model[2*ypos+1][2*xpos] = strength;
      measurement_model[2*ypos+1][2*xpos+1] = strength;

      //printf("(%.2f, %.2f) --> %f\n", offset_x * RES, offset_y * RES, strength);
      if(strength > best_strength) {
	best_x = offset_x;
	best_y = offset_y;
	best_strength = strength;
      }
    }
  }
  t2 = dgc_get_time();
  //printf("BEST coarse: (%.2f, %.2f) --> %f\n", best_x*RES, best_y*RES, best_strength);
  if(DEBUG) printf("   t2 = %f\n", t2 - t1);
  int best_x2 = best_x, best_y2 = best_y;
  for(offset_x = best_x - 1; offset_x <= best_x + 1; offset_x++) {
    for(offset_y = best_y - 1; offset_y <= best_y + 1; offset_y++) {
      double strength = align_strength(offset_x, offset_y);
      //printf("(%.2f, %.2f) --> %f\n", offset_x * RES, offset_y * RES, strength);
      measurement_model[offset_y+radius-1][offset_x+radius-1] = strength;

      if(strength > best_strength) {
	best_x2 = offset_x;
	best_y2 = offset_y;
	best_strength = strength;
      }
    }
  }
  pthread_mutex_lock(&posterior_mutex);
  int best_x3=0, best_y3=0;
  double best_posterior = -1000000000000.0;
  for(i = 0; i <= GRID_RADIUS * 2; i++) {
	for(j = 0; j <= GRID_RADIUS * 2; j++) {
		//hist_z[i][j] = hist_z0[i][j];
		posterior[i][j] *= pow(measurement_model[i][j], MEASUREMENT_WEIGHT);
	}
  }
  normalize_posterior();
  for(i = 0; i <= GRID_RADIUS * 2; i++) {
	for(j = 0; j <= GRID_RADIUS * 2; j++) {
		if(posterior[i][j] > best_posterior) {
			best_posterior = posterior[i][j];
			best_x3 = j + 1 - radius;
			best_y3 = i + 1 - radius;
			//printf("NEW BEST: %d, %d\n", best_x3, best_y3);
		}
		hist_z[i/2][j/2] = posterior[j][i];
	}
  }
  
  //printf("New best: %d, %d\n", best_x3, best_y3);
  double best_x4=0, best_y4 = 0;
  double weight_sum = 0.00000001;
  for(i = best_y3 - 2; i <= best_y3 + 2; i++) {
    for(j = best_x3 - 2; j <= best_x3 + 2; j++) {
        //printf("i, j = %d, %d\n", i, j);
	int x = j + radius - 1;
        int y = i + radius - 1;
	if(x < 0 || y < 0 || x > 2*GRID_RADIUS || y > 2*GRID_RADIUS)
		continue;
        //printf("   this is okay\n");
        double weight = pow(posterior[y][x], 2);
        weight_sum += weight;
        best_x4 += j * weight;
        best_y4 += i * weight;
    }
  }
  //printf("pre-best4: %f, %f      sum: %f\n", best_x4, best_y4, weight_sum);
  best_x4 /= weight_sum;
  best_y4 /= weight_sum;
  pthread_mutex_unlock(&posterior_mutex);
  t3 = dgc_get_time();
  if(DEBUG) printf("   t3 = %f\n", t3 - t2);
  localize_x_offset = best_x4 * RES;
  localize_y_offset = best_y4 * RES;
  printf("BEST refined: (%.2f, %.2f) --> %f\n", best_x3 * RES, best_y3 * RES, best_strength);
  printf("    BEST ultra-refined: (%.2f, %.2f)\n", localize_x_offset, localize_y_offset);
  //printf("   t_total = %f\n", t3 - t0);
}

void track_applanix(void) {
  if(!received_applanix_pose)
    return;
  
  char utmzone[10];
  pthread_mutex_lock(&applanix_pose_mutex);
  smooth_x = applanix_pose.smooth_x;
  smooth_y = applanix_pose.smooth_y;
  smooth_z = applanix_pose.smooth_z;
  yaw = applanix_pose.yaw;
  timestamp = applanix_pose.timestamp;
  double speed = applanix_pose.speed;
  vlr::latLongToUtm(applanix_pose.latitude, applanix_pose.longitude, &utm_x,
	      &utm_y, utmzone);
  pthread_mutex_unlock(&applanix_pose_mutex);
  
  static int first = 1;
  static double last_x = 0, last_y = 0;
  static double last_motion_time = 0;
  if(first) {
    pthread_mutex_lock(&velodyne_mutex);
    dgc_grid_recenter_grid(grid, smooth_x, smooth_y);
    pthread_mutex_unlock(&velodyne_mutex);
  }
  if(first || hypot(utm_x - last_x, utm_y - last_y) > 5.0) {
    //printf("Recentering tile grid\n");
    dgc_grid_recenter_grid(tile_grid, utm_x, utm_y);
    last_x = utm_x;
    last_y = utm_y;
  }
  first = 0;
  if(timestamp > last_motion_time + 0.1 && speed > .1) {
	last_motion_time = timestamp;
  	motion_model(speed);
  } 
}

void localize(void) {
  if(!received_applanix_pose)
    return;
  
  static int first = 1;
  static double last_time = 0;
  static double last_x = 0;
  static double last_y = 0;
  
  if(first || (hypot(smooth_x - last_x, smooth_y - last_y) > LOCALIZE_DISTANCE_INTERVAL &&
	       timestamp - last_time > LOCALIZE_TIME_INTERVAL)) {
	//printf("Sup yo, time dif is %f\n", timestamp - last_time);
    compute_best_offset();
    last_x = smooth_x;
    last_y = smooth_y;
    last_time = timestamp;
    first = 0;
    has_localized = 1;
  }
}

void publish(void) {
  if(!received_applanix_pose || !has_localized)
    return;
  
  static double last_time = 0, last_log_time = 0;
  static int first = 1;
  //static double last_x_offset = 0, last_y_offset = 0;
  
  if(first) {
    outlog = fopen("localizelog.txt", "w");
    if(!outlog) { 
    	fprintf(stderr, "Error: cannot open localizelog.txt for writing. Make sure you have write permission in this directory.\nDisconnecting.\n");
    	exit(0);
    }
    fprintf(outlog, "ts\tutm_x\tutm_y\tdx\tdy\tdlat\tdlon\n");
  }
  if(first || dgc_get_time() - last_time > PUBLISH_INTERVAL) {
    //printf("Actually publish!\n");
    LocalizePose pose_message;
    strcpy(pose_message.utmzone, UTM_ZONE);
    pthread_mutex_lock(&applanix_pose_mutex);
    double next_x_offset = utm_x + localize_x_offset - smooth_x;
    double next_y_offset = utm_y + localize_y_offset - smooth_y;
    if(last_x_offset) {
      next_x_offset = SMOOTHNESS * last_x_offset + (1 - SMOOTHNESS) * next_x_offset;
      next_y_offset = SMOOTHNESS * last_y_offset + (1 - SMOOTHNESS) * next_y_offset;
    }
    pose_message.x_offset = next_x_offset;
    pose_message.y_offset = next_y_offset;
    if(timestamp - last_log_time > 1.0) {  // log every second
      double dx = next_x_offset - utm_x + smooth_x;
      double dy = next_y_offset - utm_y + smooth_y;
      double d = pow(dx * dx + dy * dy, .5);
      double theta = M_PI/2 + atan2(dy, dx) - yaw;
      double dlat = d * cos(theta);
      double dlon = d * sin(theta);
      printf("lat: %.2f   lon: %.2f   mag: %.2f\n", dlat, dlon, d);
      fprintf(outlog, "%lf\t%lf\t%lf\t%f\t%f\t%f\t%f\t%f\t%f\n", timestamp, next_x_offset + smooth_x, 
	      next_y_offset + smooth_y, smooth_x + dx, smooth_y + dy, dx, dy, dlat, dlon);
      last_log_time = timestamp;
    }
    pthread_mutex_unlock(&applanix_pose_mutex);
    
    pose_message.std_x = pose_message.std_y = pose_message.std_s = 0;
    dgc_localize_publish_pose_message(ipc, &pose_message);
    
    first = 0;
    last_time = dgc_get_time();
    last_x_offset = next_x_offset;
    last_y_offset = next_y_offset;

    if(SHOW_GLS)
	plot_2D();
  }
}

void *applanix_thread(__attribute__ ((unused)) void *ptr) {
    printf("Applanix thread says hi!\n");
    while(1) {
    	track_applanix();
	usleep(1000);
    }
    return NULL;
}

void *localize_thread(__attribute__ ((unused)) void *ptr) {
  printf("Localize thread says hi!\n");
  while(1) {
    localize();
    usleep(1000);
  }
  return NULL;
}


void *velodyne_thread(__attribute__ ((unused)) void *ptr) {
  dgc_velodyne_client *velodyne;
  dgc_velodyne_spin *vspin = NULL;
  int i, j, beam_num, ring_num;
  double p_x, p_y, p_z, applanix_vel;
  map_cell_p cell;

  printf("hello velodyne thread!\n");

  velodyne = new dgc_velodyne_client(cal_filename, int_filename, velodyne_offset);
  vspin = new dgc_velodyne_spin;

  while(!received_applanix_pose)
	usleep(1000);

  /* read forever from the velodyne */
  while(1) {
    while(velodyne->scans_available()) {
	//printf("VELODYNE DATA!!\n");
      pthread_mutex_lock(&velodyne_mutex);
      velodyne->read_spin(vspin);

      /* put the intensitysity data in the rolling grid */
      if(vspin->num_scans > 0) {
		//printf("ACTUAL VELODYNE DATA!\n");
	dgc_grid_recenter_grid(grid, vspin->scans[0].robot.x,
			       vspin->scans[0].robot.y);

	pthread_mutex_lock(&applanix_pose_mutex);
	applanix_vel = applanix_pose.speed;
	pthread_mutex_unlock(&applanix_pose_mutex);

	for(i = 0; i < vspin->num_scans; i++) 
	  for(j = 0; j < VELO_BEAMS_IN_SCAN; j++) {
	    beam_num = j + vspin->scans[i].block * 32;
	    ring_num = velodyne->config->inv_beam_order[beam_num];

	    if(beam_num == 0 || beam_num == 6 || beam_num == 10 || beam_num == 24)
	      continue;
	    if(vspin->scans[i].p[j].range < 0.01)
	      continue;
	    if(fabs(applanix_vel) < dgc_mph2ms(1.0))
	      continue;

	    p_x = vspin->scans[i].p[j].x * 0.01 + vspin->scans[i].robot.x;
	    p_y = vspin->scans[i].p[j].y * 0.01 + vspin->scans[i].robot.y;
	    p_z = vspin->scans[i].p[j].z * 0.01 + vspin->scans[i].robot.z;

	    cell = (map_cell_p)dgc_grid_get_xy(grid, p_x, p_y);
	    if(cell != NULL) {
		int intensity = vspin->scans[i].p[j].intensity;
		if(intensity > 1) {
			cell->intensity_sum += intensity;
			cell->intensity_count++;
		        if(cell->n == 0) {
                		cell->n = 1;
                		cell->xa = p_z;
                		cell->sxi2 = p_z * p_z;
        		} else {
                		cell->n++;
                		cell->xa = ((cell->n - 1) * cell->xa + p_z) / cell->n;
                		cell->sxi2 += p_z * p_z;
        		}
        		cell->stdev = pow((cell->sxi2 / cell->n - cell->xa * cell->xa), .5);
		}
	    }
	  }
	grid_updated = 1;
      }
      pthread_mutex_unlock(&velodyne_mutex);
    }
    usleep(1000);
  }
  return NULL;
}

void initialize_grid(void) {
  map_cell_p default_map_cell = NULL;

  /* allocate rolling grid */
  default_map_cell = (map_cell_p)calloc(1, sizeof(map_cell_t));
  dgc_test_alloc(default_map_cell);
  default_map_cell->intensity_sum = 0;
  default_map_cell->intensity_count = 0;

  grid = dgc_grid_initialize(RES, DIM, DIM, sizeof(map_cell_t),
                             default_map_cell);
  fprintf(stderr, "Grid memory required: %.2f MB\n", 
	  grid->rows * grid->cols * grid->bytes_per_cell / 1024.0 / 1024.0);
}

void initialize_tile_grid(void) {
  grid_cell_t *default_tile = NULL;
  default_tile = (grid_cell_t *)calloc(1, sizeof(grid_cell_t));
  tile_grid = dgc_grid_initialize(TERRAIN_TILE_RESOLUTION * TERRAIN_TILE_SIZE,
                                  TERRAIN_GRID_NUM_TILES, TERRAIN_GRID_NUM_TILES,
                                  sizeof(grid_cell_t), default_tile);
  dgc_grid_set_clear_handler(tile_grid, tile_clear_handler);
}

void applanix_pose_handler(void) {
  //printf("applanix!\n");
  received_applanix_pose = 1;
  track_applanix();
  publish();
}

void 
read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = {
    {"transform", "velodyne", DGC_PARAM_TRANSFORM, &velodyne_offset, 1, NULL},
    {"velodyne", "cal_file", DGC_PARAM_FILENAME, &cal_filename, 0, NULL},
    {"imagery", "root", DGC_PARAM_FILENAME, &imagery_root, 0, NULL},
    {"velodyne", "int_file", DGC_PARAM_FILENAME, &int_filename, 0, NULL},
    {"velodyne", "calibrate_intensities", DGC_PARAM_INT, &calibrate_intensities, 0, NULL},
  };
  pint->InstallParams(argc, argv, 
		      params, sizeof(params)/sizeof(params[0]));
  if(!calibrate_intensities) {
	printf("WARNING: not calibrating Velodyne intensities.\n");
    int_filename = NULL;
  }
}

void 
dgc_localize_register_ipc_messages(IpcInterface *ipc) 
{
  IpcMessageID messages[] = { LocalizePoseID };
  ipc->DefineMessageArray(messages, sizeof(messages) / sizeof(*messages));
}

void 
dgc_localize_publish_pose_message(IpcInterface *ipc, LocalizePose *pose) 
{
  int   err;
  strncpy(pose->host, dgc_hostname(), 10);
  pose->timestamp   = dgc_get_time();
  err = ipc->Publish(LocalizePoseID, pose);
  TestIpcExit(err, "Could not publish", LocalizePoseID);
}

void shutdown_localize_module(int sig) {
  if(sensor_data)
    free(sensor_data);
  if(sig == SIGINT) {
    if(outlog)
	fclose(outlog);
    fprintf(stderr, "\nDisconnecting.\n");
    exit(0);
  }
}


int 
main(int argc, char **argv) {
  pthread_t thread_velodyne, thread_localize;

  initialize_grid();
  initialize_tile_grid();
  posterior_init();

  /* handle IPC messages */
  ipc = new dgc::IpcStandardInterface;
  if (ipc->ConnectLocked("localize_velodyne") < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  dgc_localize_register_ipc_messages(ipc);

  dgc::ParamInterface  *pint;
  pint = new dgc::ParamInterface(ipc);
  read_parameters(pint, argc, argv);
  init_colors();
  init_guassians();
  gls = gls_alloc("LOCALIZE");

  sensor_data = (map_cell_t*)malloc(DIM * DIM * sizeof(map_cell_t));
  pthread_create(&thread_velodyne, NULL, velodyne_thread, NULL);
  pthread_create(&thread_localize, NULL, localize_thread, NULL);

  ipc->Subscribe(ApplanixPoseID, &applanix_pose,
		 applanix_pose_handler,
		 DGC_SUBSCRIBE_LATEST,
		 &applanix_pose_mutex);

  signal(SIGINT, shutdown_localize_module);
  ipc->Dispatch();
  return 0;
}
