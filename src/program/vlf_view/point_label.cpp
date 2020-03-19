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
#include <iostream>
#include <limits>

#include <graph_cut.h>
#include <grid.h>

#include <ext/hash_set>

#define LEFT	1	
#define	RIGHT	2
#define FRONT	3	
#define BACK	4	

using namespace dgc;
using namespace vlr;

using std::vector;

typedef std::vector<dgc_velodyne_point_p> laser_cluster_t;
typedef laser_cluster_t* laser_cluster_p;

typedef struct {
  unsigned short          hits;
  unsigned short          seen;

  unsigned char           obstacle;
  unsigned char           dynamic;
  float                   min;
  float                   max;
  unsigned char           street;

  /* region in segmentation */
  unsigned short          region;

  /* counter compare modified for the obstacles list */
  unsigned short          last_use;

  /* counter compare modified for the map diff list */
  unsigned short          last_mod;

  unsigned short          last_min;
  unsigned short          last_max;

} dgc_perception_map_cell_t, *dgc_perception_map_cell_p;

typedef struct {
  dgc_velodyne_scan_p        scan;
  dgc_velodyne_point_p       point;
  dgc_velodyne_point_p       label;
  dgc_perception_map_cell_p  cell;
  int                        encoder;
  unsigned char              obstacle;
  float                      delta_range;
  float                      slope;
  float                      slope2;
  float                      local_z;
  unsigned char              valid;
} laser_point_t, *laser_point_p;

typedef struct {
  int                      num_points;
  laser_point_t          * laser_point;
  dgc_pose_t             * robot;
} laser_scan_t, *laser_scan_p;

float wrap(float t) {
  while(t > M_PI)
    t -= 2*M_PI;
  while(t < -M_PI)
    t += 2*M_PI;
  return t;
}

/* parameters */
char *imagery_root;
char *cal_filename = NULL;
dgc_transform_t velodyne_offset;
char vlf_filename[300], labels_filename[300], labels_ext[300], features_filename[300];

/* velodyne stuff */
dgc_velodyne_file_p velodyne_file = NULL;
dgc_velodyne_index velodyne_index, label_index;
dgc_velodyne_config_p velodyne_config = NULL;
dgc_velodyne_spin spin, labels_spin, test_spin;
bool labelled = false;

dgc_grid_p terrain_grid = NULL;

vector<dgc_velodyne_spin> labeled_spins;
vector<laser_cluster_p> clusters;

int current_spin_num = 0;

#define LABEL_UNKNOWN  0
#define LABEL_GROUND   1
#define LABEL_OBSTACLE 2
#define LABEL_NOISE    3

int current_label = LABEL_UNKNOWN;

/* graphics */

dgc_passatwagonmodel_t* passat = NULL;
int large_points = 1;
int color_mode = 2;
int draw_flat = 0;
int show_clusters = 0;
int show_spin = 1;
int show_grid = 0;
int show_rect = 0;
int last_mouse_x = 0, last_mouse_y = 0;


double get_z() {
  return spin.scans[0].robot.z;
}

static unsigned char colors[17][3] = {
  {0,0,255},
  {0,255,0},
  {255,0,0},
  {255,255,0},
  {0,255,255},
  {255,0,255},
  {255,255,255},
  {0,0,0},
  {128,51,128},
  {51,128,128},
  {255,51,51},
  {51,255,51},
  {51,51,255},
  {51,179,204},
  {128,255,51},
  {255,128,51},
  {51,128,255}
};

/* rectangle for selecting points */
int rect_x = 0;
int rect_y = 0;
int rect_x2 = 0;
int rect_y2 = 0;

#define     MODE_NONE                0
#define     MODE_SELECT_CENTER       1
#define     MODE_SELECT_FRONT        2
#define     MODE_SELECT_SIDE         3
#define     MODE_SELECT_CORNER       4

int edit_mode = MODE_NONE;
int show_distance = 0;

double ground_threshold = 0.2;

/* tracker stuff */
GlsOverlay                    * gls = NULL;

LocalizePose                    localize_pose = {0, 0.0, 0.0, "", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "" };


char                          * rndf_filename = NULL;

/* features */
typedef struct {
  int     idx;      /* velodyne index beam */
  int     pb;       /* partner beam */
  int     next;     /* next beam */
  float   fac;      /* approximation factor of function with pb */
  float   v_angle;  /* vertical angle of the laser beam */
  float   h_angle;  /* horizontal angle of the laser beam */
  int     h_offset; /* horizontal offset of the beam in velodyne ticks */
} velodyne_ring_features_t;

#define    NUM_LASER_BEAMS 64
#define    MAX_POINTS_PER_SCAN      8000

#define NUM_SAMPLES    1000
#define EPSILON        0.00001
#define MAX_RANGE      70.0

#define    VELO_BLIND_SPOT_START    17000
#define    VELO_BLIND_SPOT_STOP     19000
#define    MAX_BEAMS_IN_BIN         1000
#define    BINS_PER_REV             720

int                       min_h_offset;

velodyne_ring_features_t  rings[NUM_LASER_BEAMS];
int                       ringsIdx[NUM_LASER_BEAMS];
laser_scan_t*             laser_scan = NULL;
laser_scan_t*             next_laser_scan = NULL;
//laser_scan_t              laser_scan[NUM_LASER_BEAMS];
//laser_scan_t              next_laser_scan[NUM_LASER_BEAMS];


typedef struct {
  float  x;
  float  y;
} sample_t;

typedef struct {
  int                 num_beams;
  int                 encoder;
  laser_point_p     * beam;
} beam_bin_t;

beam_bin_t           bins[NUM_LASER_BEAMS][720];

void cluster_center(laser_cluster_p cluster, double* x, double* y)
{
  *x = 0;
  *y = 0;
  for (unsigned int i=0; i < cluster->size(); i++) {
    *x += cluster->at(i)->x;
    *y += cluster->at(i)->y;
  }
  *x = (*x / cluster->size() * 0.01) + spin.scans[0].robot.x;
  *y = (*y / cluster->size() * 0.01) + spin.scans[0].robot.y;
}

void cluster_size(laser_cluster_p cluster, double* w, double* l)
{
  short min_x = 1e4;
  short min_y = 1e4;
  short max_x = -1e4;
  short max_y = -1e4;

  for (unsigned int i=0; i < cluster->size(); i++) {
    min_x = std::min(min_x, cluster->at(i)->x);
    max_x = std::max(max_x, cluster->at(i)->x);
    min_y = std::min(min_y, cluster->at(i)->y);
    max_y = std::max(max_y, cluster->at(i)->y);
  }

  *w = (max_x - min_x) * 0.01;
  *l = (max_y - min_y) * 0.01;

}

void label_all_points(dgc_velodyne_spin *vspin, unsigned char label) {
  for(int i = 0; i < vspin->num_scans; i++) {
    for(int j = 0; j < VELO_BEAMS_IN_SCAN; j++) {
      vspin->scans[i].p[j].intensity = label;
    }
  }
}

void label_selected_points(dgc_velodyne_spin *vspin, dgc_velodyne_spin *labels, unsigned char label, double origin_x, double origin_y, double origin_z, int flat) {
  GLdouble model[16];
  GLdouble proj[16];
  GLint view[4];

  // guarantee that we're extracting the 3D transformations
  gui3D_switch_to_3D_mode();

   /* extract transformations */
  glPushMatrix();
  // match the drawing transformations...
  glGetDoublev(GL_MODELVIEW_MATRIX, model);
  glGetDoublev(GL_PROJECTION_MATRIX, proj);
  glGetIntegerv(GL_VIEWPORT, view);
  int min_x, max_x, min_y, max_y;
  double win_x, win_y, win_z;

  if (rect_x < rect_x2) {
    min_x = rect_x;
    max_x = rect_x2;
  } else {
    min_x = rect_x2;
    max_x = rect_x;
  }

  if (rect_y < rect_y2) {
    min_y = rect_y;
    max_y = rect_y2;
  } else {
    min_y = rect_y2;
    max_y = rect_y;
  }

  for(int i = 0; i < vspin->num_scans; i++) {
    for(int j = 0; j < VELO_BEAMS_IN_SCAN; j++) {
      if ((label != LABEL_UNKNOWN) && (labels->scans[i].p[j].intensity != LABEL_UNKNOWN))
        continue;

      double x = vspin->scans[i].p[j].x * 0.01 + vspin->scans[i].robot.x;
      double y = vspin->scans[i].p[j].y * 0.01 + vspin->scans[i].robot.y;
      double z = vspin->scans[i].p[j].z * 0.01 + vspin->scans[i].robot.z;

      if (z - origin_z < ground_threshold)
        continue;

      x -= origin_x;
      y -= origin_y;
      z = (flat) ? 0 : ((z-origin_z<0.05) ? 0.05 : z - origin_z);

      gluProject(x, y, z,
                 model, proj, view,
                 &win_x, &win_y, &win_z);

      if ( (win_x > min_x) && (win_x < max_x) && (win_y > min_y) && (win_y < max_y) ) {
        labels->scans[i].p[j].intensity = label;
      }

    }

  }

  glPopMatrix();
}

int ringsCompare( const void *a, const void *b )
{
  static velodyne_ring_features_t v1, v2;
  v1 = *(velodyne_ring_features_t *)a;
  v2 = *(velodyne_ring_features_t *)b;
  if (v1.v_angle<v2.v_angle) {
    return(1);
  } else if (v1.v_angle==v2.v_angle) {
    return(0);
  } else {
    return(-1);
  }
}

float
eval_function( float factor, sample_t *samples, int num_samples )
{
  int     i;
  double  v, sum=0.0;

  for (i=0; i<num_samples; i++) {
    v = factor * samples[i].x * samples[i].x;
    sum += fabs(v-samples[i].y);
  }

  return( sum / (float)num_samples );
}

float
approx_with_square_function( sample_t *samples, int num_samples )
{
  int        loop;
  double     stepsize, best, test_plus, test_minus, fac;

  loop      = 1;
  stepsize  = 1.0;
  fac       = 10.0;

  best = eval_function( fac, samples, num_samples );

  while (loop && stepsize>EPSILON) {
    test_plus  = eval_function( fac+stepsize, samples, num_samples );
    test_minus = eval_function( fac-stepsize, samples, num_samples );
    if (test_plus<best) {
      fac += stepsize;
      best = test_plus;
    } else if (test_minus<best) {
      best = test_minus;
      fac -= stepsize;
    } else {
      stepsize /= 2.0;
    }
  }

  return(fac);

}

double velodyne_min_beam_diff  = 0.50;
double velodyne_threshold_factor = 0.0002;
double velodyne_max_range = 60.0;

bool initialized_features = false;
void initialize_features() {
  if (initialized_features)
    return;
  initialized_features = true;

  min_h_offset = 0;
  for (int i=0; i<NUM_LASER_BEAMS; i++) {
    rings[i].idx      = i;
    rings[i].pb       = i;
    rings[i].next     = i;
    rings[i].v_angle  = velodyne_config->vert_angle[i];
    rings[i].h_angle  = velodyne_config->rot_angle[i];
    rings[i].h_offset = (int) (VELO_NUM_TICKS*(-rings[i].h_angle/(2*M_PI)));
    min_h_offset = std::min(min_h_offset,  rings[i].h_offset);
  }
  printf("min_offset:  %d\n", min_h_offset);

  qsort( rings, NUM_LASER_BEAMS, sizeof(velodyne_ring_features_t), ringsCompare );

  sample_t sample[NUM_SAMPLES];

  /* beam 0 is highest laser - beam 63 is lowest laser */
  for (int i=0; i<NUM_LASER_BEAMS; i++) {
    ringsIdx[rings[i].idx] = i;
    /* find partner beam */
    rings[i].pb = i;
    float dist = 0.0;
    float a1,a2;
    a2 = a1 = M_PI_2+rings[i].v_angle;
    while (rings[i].pb>0 && dist<velodyne_min_beam_diff) {
      rings[i].pb--;
      a2 = M_PI_2+rings[rings[i].pb].v_angle;
      if (a2<0.0) {
        dist = velodyne_min_beam_diff;
      } else {
        dist = (DGC_PASSAT_HEIGHT + 0.8) * (tan(a2)-tan(a1));
      }
    }

    /* compute samples for distance difference function
         between the two beams */
    if (rings[i].pb!=i) {
      float a_dist = rings[i].v_angle - rings[rings[i].pb].v_angle;
      float h = DGC_PASSAT_HEIGHT + 0.7;
      for (int j=0; j<NUM_SAMPLES; j++) {
        float d = j * MAX_RANGE / (float) NUM_SAMPLES;
        sample[j].x = d;
        sample[j].y = (d-h) / sin( asin(h/d)+a_dist );
        if (isnan(sample[j].y))
          sample[j].y = 0;
      }

      /* compute best factor of square function to approximate the
     distance difference function */
      rings[i].fac = approx_with_square_function( sample, NUM_SAMPLES );
    }
  }

  for (int i=0; i<NUM_LASER_BEAMS-1; i++)
    rings[i].next = rings[i+1].idx;

  for (int l=0; l<NUM_LASER_BEAMS; l++) {
    for (int i=0; i<BINS_PER_REV; i++) {
      bins[l][i].num_beams = 0;
      bins[l][i].beam      =
          (laser_point_p *) malloc( MAX_BEAMS_IN_BIN * sizeof(laser_point_p) );
    }
  }

  laser_scan = (laser_scan_t*) malloc( NUM_LASER_BEAMS * sizeof(laser_scan_t) );
  next_laser_scan = (laser_scan_t*) malloc( NUM_LASER_BEAMS * sizeof(laser_scan_t) );
  for (int l=0; l<NUM_LASER_BEAMS; l++) {
    laser_scan[l].num_points = 0;
    laser_scan[l].laser_point =
        (laser_point_p) malloc( MAX_POINTS_PER_SCAN * sizeof(laser_point_t) );

    next_laser_scan[l].num_points = 0;
    next_laser_scan[l].laser_point =
            (laser_point_p) malloc( MAX_POINTS_PER_SCAN * sizeof(laser_point_t) );
  }
}

unsigned short min_ground_range[] =
{
    65000, 65000, 65000, 65000, 65000, 65000, 65000, 4559,
    4022,  3718,  3336,  3000,  2711,  2479,  2287,  2151,
    2004,  1921,  1842,  1741,  1657,  1590,  1510,  1380,
    1306,  1258,  1265,  1238,  1078,  1045,   983,   944,
    917,   829,   810,   759,   756,   687,   684,   622,
    630,   569,   594,   539,   539,   494,   515,   462,
    471,   427,   436,   401,   436,   380,   401,   363,
    377,   343,   354,   333,   347,   321,   332,   308
};

unsigned short max_ground_range = 7000; // 70 m

void bin_points()
{
  int l, i, e, b, n;

  // *************************************************************
  // * Put beams in bins
  // *************************************************************
  for (l=0; l<NUM_LASER_BEAMS; l++) {
    for (i=0; i<BINS_PER_REV; i++) {
      bins[l][i].num_beams = 0;
      bins[l][i].encoder = VELO_NUM_TICKS + 1;
    }
  }

  for(l=0; l<NUM_LASER_BEAMS; l++) {
    for(i=0; i<laser_scan[l].num_points; i++) {
      if (laser_scan[l].laser_point[i].valid) {
        e = laser_scan[l].laser_point[i].encoder;
        b = (int)floor(e / ((float)VELO_NUM_TICKS/(float)BINS_PER_REV));
        n = bins[l][b].num_beams;
        if (n<MAX_BEAMS_IN_BIN) {
          bins[l][b].beam[n] = &laser_scan[l].laser_point[i];

//          int encoder = laser_scan[l].laser_point[i].scan->encoder;
//          if (bins[l][b].encoder == (VELO_NUM_TICKS + 1)) {
//            bins[l][b].encoder = encoder;
//          } else if (encoder < VELO_SPIN_START) {
//            bins[l][b].encoder = std::min(encoder, bins[l][b].encoder);
//          } else {
//            bins[l][b].encoder = std::max(encoder, bins[l][b].encoder);
//          }

//          bins[l][b].beam[n]->encoder = e;
          bins[l][b].num_beams++;
        }
      }
    }
  }

}

#define NUM_GROUND_THRESHOLDS 4
//float thresholds[] = { 0.000243, 0.2, 0.8 };
//float thresholds[] = { 0.0023, 0.1, 0.15, 0.2, 1.5, 1.5 };
//float thresholds[] = { 0.0023,  0.100000,  0.150000,  0.200000,  2.000000,  1.500000 };
//float thresholds[] = { 0.0023,  0.100000,  0.150000,  0.200000,  9.500000,  4.0000  };
//float thresholds[] = { 0.0023,  0.100000,  0.100000,  0.200000,  0.500000,  2.0000  };
//float thresholds[] = { 0.002300,  0.100000,  0.100000,  0.200000,  0.548828,  5.125000 };
//float thresholds[] = { 0.002300,  0.100000,  0.100000,  0.548828,  5.125000 };
//float thresholds[] = { 0.002300,  0.100000,  0.100000,  0.548828,  5.125000 };
//float thresholds[] = { 0.0002,  0.100000,  0.100000,  0.548828,  5.125000 };
float ground_thresholds[] = { 0.001326,  0.2,  0.8,  1.0,  0.5 };

void calc_local_z()
{
  dgc_grid_clear(terrain_grid);

//  __gnu_cxx::hash_set<int> open;

  for (int l=0; l<NUM_LASER_BEAMS; l++) {
    for (int i = 0; i < laser_scan[l].num_points; i++) {
      if (!laser_scan[l].laser_point[i].valid)
        continue;
      double x = laser_scan[l].laser_point[i].point->x * 0.01;
      double y = laser_scan[l].laser_point[i].point->y * 0.01;
      short z = laser_scan[l].laser_point[i].point->z;
      short* grid_z;
      grid_z = (short*)dgc_grid_get_xy(terrain_grid, x, y);
      if (grid_z) {
        *grid_z = std::min(*grid_z, z);
//        open.insert((int)grid_z);
      }
    }
  }

//  int r0, c0;
//  short ground_z = (short)((0.2-DGC_PASSAT_HEIGHT) / 0.01);
//  dgc_grid_xy_to_rc_local(terrain_grid, 0.0, 0.0, &r0, &c0);
//  for (int r = r0-2; r < r0+3; r++) {
//    for (int c= c0-2; c < c0+3; c++) {
//      short* z = (short*)dgc_grid_get_rc_local(terrain_grid, r, c);
//      *z = std::min(*z, ground_z);
//      open.insert((int)z);
//    }
//  }

//  while (open.size() > 0) {
//    short* cell = (short*) *open.begin();
////    printf("processing %d %d\n", (int)cell, *cell);
//    int r,c;
//    dgc_grid_cell_to_rc_local(terrain_grid, cell, &r, &c);
//
//    short min_neighborhood_z = *cell;
//    // look at four neighborhood
//    short* neighbor = (short*)dgc_grid_get_rc_local(terrain_grid, r+1, c);
//    if (neighbor) min_neighborhood_z = std::min(min_neighborhood_z, *neighbor);
//
//    neighbor = (short*)dgc_grid_get_rc_local(terrain_grid, r-1, c);
//    if (neighbor) min_neighborhood_z = std::min(min_neighborhood_z, *neighbor);
//
//    neighbor = (short*)dgc_grid_get_rc_local(terrain_grid, r, c-1);
//    if (neighbor) min_neighborhood_z = std::min(min_neighborhood_z, *neighbor);
//
//    neighbor = (short*)dgc_grid_get_rc_local(terrain_grid, r, c+1);
//    if (neighbor) min_neighborhood_z = std::min(min_neighborhood_z, *neighbor);
//
////    printf("min_neigbor_z: %d\n", min_neighborhood_z);
//    if (*cell > (min_neighborhood_z + 10)) {
////      printf("lowering %d to %d (%d + %d)\n", *cell, min_neighborhood_z + 20, min_neighborhood_z, 20);
//      *cell = min_neighborhood_z + 10;
////      printf("new value: %d  %d\n", *(short*) dgc_grid_get_rc_local(terrain_grid, r, c), *cell);
//      neighbor = (short*)dgc_grid_get_rc_local(terrain_grid, r+1, c);
//      if (neighbor && (*neighbor > (min_neighborhood_z + 20))) open.insert((int)neighbor);
//      neighbor = (short*)dgc_grid_get_rc_local(terrain_grid, r-1, c);
//      if (neighbor && (*neighbor > (min_neighborhood_z + 20))) open.insert((int)neighbor);
//      neighbor = (short*)dgc_grid_get_rc_local(terrain_grid, r, c-1);
//      if (neighbor && (*neighbor > (min_neighborhood_z + 20))) open.insert((int)neighbor);
//      neighbor = (short*)dgc_grid_get_rc_local(terrain_grid, r, c+1);
//      if (neighbor && (*neighbor > (min_neighborhood_z + 20))) open.insert((int)neighbor);
//    }
//
//    open.erase((int)cell);
//
////    printf("%d size\n", open.size());
//  }

  for (int l=0; l<NUM_LASER_BEAMS; l++) {
    for (int i = 0; i < laser_scan[l].num_points; i++) {
      double x = laser_scan[l].laser_point[i].point->x * 0.01;
      double y = laser_scan[l].laser_point[i].point->y * 0.01;
      short z = laser_scan[l].laser_point[i].point->z;
      short* grid_z;
      grid_z = (short*)dgc_grid_get_xy(terrain_grid, x, y);
      if (grid_z)
        laser_scan[l].laser_point[i].local_z = (z - *grid_z) * 0.01;
    }
  }
}

void calc_slopes()
{
  const float max_slope = 10.0;

  for (int l=0; l<NUM_LASER_BEAMS; l++) {
    int l0 = rings[l].idx;

    if (ringsIdx[l0]+1 >= NUM_LASER_BEAMS)
      continue;
    int l1 = rings[ringsIdx[l0]+1].idx;

    for (int i = 0; i < laser_scan[l0].num_points; i++) {
      float dz = abs(laser_scan[l1].laser_point[i].point->z - laser_scan[l0].laser_point[i].point->z) * 0.01;
      float dx = (laser_scan[l1].laser_point[i].point->x - laser_scan[l0].laser_point[i].point->x) * 0.01;
      float dy = (laser_scan[l1].laser_point[i].point->y - laser_scan[l0].laser_point[i].point->y) * 0.01;
      float delta_d = sqrt(dx*dx + dy*dy);

      if (delta_d > EPSILON) {
        laser_scan[l0].laser_point[i].slope = laser_scan[l1].laser_point[i].slope2 = std::min(max_slope, dz / delta_d);
      } else if (dz > EPSILON) {
        laser_scan[l0].laser_point[i].slope = laser_scan[l1].laser_point[i].slope2 = max_slope;
      }
    }

//    if (ringsIdx[l0]+2 >= NUM_LASER_BEAMS)
//      continue;
//    l1 = rings[ringsIdx[l0]+2].idx;
//
//    for (int i = 0; i < laser_scan[l0].num_points; i++) {
//      float dz = abs(laser_scan[l1].laser_point[i].point->z - laser_scan[l0].laser_point[i].point->z) * 0.01;
//      float dx = (laser_scan[l1].laser_point[i].point->x - laser_scan[l0].laser_point[i].point->x) * 0.01;
//      float dy = (laser_scan[l1].laser_point[i].point->y - laser_scan[l0].laser_point[i].point->y) * 0.01;
//      float delta_d = sqrt(dx*dx + dy*dy);
//
//      if (delta_d > EPSILON)
//        laser_scan[l0].laser_point[i].slope2 = std::min(max_slope, dz / delta_d);
//      else if (dz > EPSILON)
//        laser_scan[l0].laser_point[i].slope2 = max_slope;
//    }
//
//    if (ringsIdx[l0]+3 >= NUM_LASER_BEAMS)
//      continue;
//    l1 = rings[ringsIdx[l0]+3].idx;
//
//    for (int i = 0; i < laser_scan[l0].num_points; i++) {
//      float dz = abs(laser_scan[l1].laser_point[i].point->z - laser_scan[l0].laser_point[i].point->z) * 0.01;
//      float dx = (laser_scan[l1].laser_point[i].point->x - laser_scan[l0].laser_point[i].point->x) * 0.01;
//      float dy = (laser_scan[l1].laser_point[i].point->y - laser_scan[l0].laser_point[i].point->y) * 0.01;
//      float delta_d = sqrt(dx*dx + dy*dy);
//
//      if (delta_d > EPSILON)
//        laser_scan[l0].laser_point[i].slope3 = std::min(max_slope, dz / delta_d);
//      else if (dz > EPSILON)
//        laser_scan[l0].laser_point[i].slope3 = max_slope;
//    }
  }
}

void label_obstacle_points_badly()
{
  int l, b, k, m, l0, l1;
  float threshold, range0, range1;

  for (l=0; l<NUM_LASER_BEAMS-1; l++) {
    if (l!=rings[l].pb) {
      l1 = rings[l].idx;            /* shorter beam */
      l0 = rings[rings[l].pb].idx;   /* longer beam */
      for( b = 0; b < BINS_PER_REV; b++) {
        for( k=0; k<bins[l0][b].num_beams; k++) {
          range0 = bins[l0][b].beam[k]->point->range;
          if (bins[l0][b].beam[k]->valid && range0<velodyne_max_range*100.0) {
            for( m=0; m<bins[l1][b].num_beams; m++) {
              if (bins[l1][b].beam[m]->valid) {
                range1 = bins[l1][b].beam[m]->point->range;
                threshold = rings[l].fac * 0.01 * range1 * range1 * velodyne_threshold_factor;
                bins[l1][b].beam[m]->delta_range = fabs(range1-range0); // = fabs(delta_z / delta_d);
                if ( fabs(range1-range0)<threshold ) {
                  bins[l1][b].beam[m]->obstacle = TRUE;
                  break;
                }
              }
            }
          }
        }
      }
    }
  }
}

void label_obstacle_points()
{
  int l, b, k, m, l0, l1;
  float threshold, range0, range1;

  for (l=0; l<NUM_LASER_BEAMS-1; l++) {
    if (l!=rings[l].pb) {
      l1 = rings[l].idx;            /* shorter beam */
      l0 = rings[rings[l].pb].idx;   /* longer beam */
      for( b = 0; b < BINS_PER_REV; b++) {
//        if ((bins[l0][b].encoder < VELO_SPIN_START) != (bins[l1][b].encoder < VELO_SPIN_START))
//          continue;
        for( k=0; k<bins[l0][b].num_beams; k++) {
          range0 = bins[l0][b].beam[k]->point->range;
          if (bins[l0][b].beam[k]->valid && range0<velodyne_max_range*100.0) {
            for( m=0; m<bins[l1][b].num_beams; m++) {
              if (bins[l1][b].beam[m]->valid) {
                range1 = bins[l1][b].beam[m]->point->range;
                threshold = rings[l].fac * 0.01 * range1 * range1 * ground_thresholds[0]; //velodyne_threshold_factor;
                bins[l1][b].beam[m]->delta_range = fabs(range1-range0); // = fabs(delta_z / delta_d);
                if ( fabs(range1-range0)<threshold ) {
                  bins[l0][b].beam[k]->obstacle = bins[l1][b].beam[m]->obstacle = TRUE;
//                  break;
                }
              }
            }
          }
        }
      }
    }
  }
}


#define NUM_GROUND_WEIGHTS 5
//unsigned int weights[] = {1, 0, 10, 10, 0, 0, 1, 1 };
//unsigned int weights[] = {1, 0, 5, 5, 0, 0, 4, 0 };
//unsigned int weights[] = {1, 0, 9, 9, 0, 1, 9, 0 };
//unsigned int weights[] = {1, 0, 19, 7, 0, 0, 12, 0 };
//unsigned int weights[] = {1, 0, 19, 7, 5, 5, 12, 0 };
//unsigned int weights[] = {0, 4, 18, 7, 5, 0, 12, 0 };
//unsigned int weights[] = {8, 4, 18, 6, 4, 0, 8, 1 };
//unsigned int weights[] = {8, 4, 18, 6, 4, 0, 8, 0, 1, 0 };
//unsigned int weights[] = {2, 4, 11, 6, 4, 0, 16, 1, 1, 1 };
//unsigned int weights[] = {3, 4, 16, 9, 4, 0, 18, 1, 1, 1 };
//unsigned int weights[] = {10, 2, 19, 8, 0, 2, 2, 16, 0, 3 };
//unsigned int weights[] = {10, 2, 19, 8, 0, 0, 2, 16, 0, 3 };
//unsigned int weights[] = {10, 2, 19, 8, 0, 0, 2, 16, 0, 3 };
//unsigned int weights[] =   {6, 2, 17, 16, 0, 2, 4, 12, 0, 3 };
//unsigned int weights[] =   {6, 2, 17, 16, 1, 1, 1, 4, 12, 0, 3 };
//unsigned int weights[] =   {17, 2, 19, 30, 12, 2, 2, 11, 1, 5, 5 };
//unsigned int weights[] =   {17, 2, 19, 30, 12, 2, 2, 11, 1, 5, 5 };
//unsigned int weights[] = {7, 2, 10, 18, 14, 0, 0, 18, 15, 4, 5 };
//unsigned int weights[] = {7, 2, 10, 40, 7, 1, 6, 19, 19, 4, 8 };
//unsigned int weights[] = {30, 2, 39, 27, 8, 3, 7, 39, 34, 12, 13 };
//unsigned int weights[] = {30, 2, 36, 96, 14, 0, 0, 94, 58, 12, 12 };
//unsigned int weights[] = {22, 2, 102, 100, 48, 48, 0, 172, 102, 4, 12 };
//unsigned int weights[] = {42, 2, 72, 80, 48, 48, 96, 5, 5, 96, 5, 5, 22 };
//unsigned int weights[] = {64, 2, 98, 96, 0, 64, 14, 0, 0, 18, 0, 0 };
//unsigned int weights[] = {64, 2, 98, 98, 0, 64, 96, 0, 66, 10, 0, 0 };
//unsigned int weights[] = {62, 2, 98, 96, 16, 62, 92, 0, 66, 12, 0, 0, };
//unsigned int weights[] = {98, 48, 98, 50, 62, 72, 0, 2, 12, 12, 0, 0, }
//unsigned int weights[] = {88, 12, 90, 98, 86, 88, 86, 0, 22, 0, 0, 4, }; // score
//unsigned int weights[] = {64, 28, 66, 36, 80, 56, 22, 2, 0, 2, 0, 6, }; // score: 0.9919
unsigned int ground_weights[] = {1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0 };


void process_laser_scan(dgc_velodyne_spin *vspin, dgc_velodyne_spin* labels) {
  for (int l=0; l<NUM_LASER_BEAMS; l++) {
    laser_scan[l].num_points = 0;
  }

  laser_scan_t* tmp = laser_scan;
  laser_scan = next_laser_scan;
  next_laser_scan = tmp;

  for(int i = 0; i < vspin->num_scans; i++) {
    int encoder = (vspin->scans[i].encoder + VELO_SPIN_START) % VELO_NUM_TICKS; // unwrap back to 0 ... 36000 range
    encoder -= min_h_offset;
    for(int j = 0; j < VELO_BEAMS_IN_SCAN; j++) {
      int l = j + vspin->scans[i].block * 32;
      int e = encoder + rings[ringsIdx[l]].h_offset;

      if (e < 0) {
        fprintf(stderr, "#ERROR: e should never be less than zero: %d", e);
      }

      if (e >= VELO_NUM_TICKS) {
        e -= VELO_NUM_TICKS;
        int n = next_laser_scan[l].num_points;
        if (n<MAX_POINTS_PER_SCAN) {
          next_laser_scan[l].laser_point[n].scan       = &(vspin->scans[i]);
          next_laser_scan[l].laser_point[n].point      = &(vspin->scans[i].p[j]);
          next_laser_scan[l].laser_point[n].label      = &(labels->scans[i].p[j]);
          next_laser_scan[l].laser_point[n].obstacle   = 0;
          next_laser_scan[l].laser_point[n].slope      = 0;
          next_laser_scan[l].laser_point[n].slope2     = 0;
          next_laser_scan[l].laser_point[n].local_z    = 0;
          next_laser_scan[l].laser_point[n].delta_range= 0;
          next_laser_scan[l].laser_point[n].encoder    = e;
          next_laser_scan[l].laser_point[n].valid      = velodyne_config->laser_enabled[l];
          next_laser_scan[l].num_points++;
        }
        next_laser_scan[l].robot = &(vspin->scans[i].robot);

      } else {
//        if (e<0) e += VELO_NUM_TICKS;
//        if (e>=VELO_NUM_TICKS) e -= VELO_NUM_TICKS;
        int n = laser_scan[l].num_points;
        if (n<MAX_POINTS_PER_SCAN) {
          laser_scan[l].laser_point[n].scan       = &(vspin->scans[i]);
          laser_scan[l].laser_point[n].point      = &(vspin->scans[i].p[j]);
          laser_scan[l].laser_point[n].label      = &(labels->scans[i].p[j]);
          laser_scan[l].laser_point[n].obstacle   = 0;
          laser_scan[l].laser_point[n].slope      = 0;
          laser_scan[l].laser_point[n].slope2     = 0;
          laser_scan[l].laser_point[n].local_z    = 0;
          laser_scan[l].laser_point[n].delta_range= 0;
          laser_scan[l].laser_point[n].encoder    = e;
          laser_scan[l].laser_point[n].valid      = velodyne_config->laser_enabled[l];
          laser_scan[l].num_points++;
        }
        laser_scan[l].robot = &(vspin->scans[i].robot);
      }
    }
  }
}

void segment_points_badly(dgc_velodyne_spin *vspin, dgc_velodyne_spin* labels) {
  initialize_features();

  process_laser_scan(vspin, labels);
  bin_points();
  label_obstacle_points_badly();

  int scan_width = laser_scan[0].num_points;

  for(int i = 0; i < NUM_LASER_BEAMS; i++) {
    for(int j = 0; j < scan_width; j++) {
      int l = i;
//      if (!laser_scan[l].laser_point[j].valid)
//        continue;

      if (laser_scan[l].laser_point[j].obstacle)
        laser_scan[l].laser_point[j].label->intensity = LABEL_OBSTACLE;
      else
        laser_scan[l].laser_point[j].label->intensity = LABEL_GROUND;
    }
  }
}

void segment_points(dgc_velodyne_spin *vspin, dgc_velodyne_spin* labels) {
  initialize_features();

  double map_width = 100.0;
  double map_resolution = ground_thresholds[3];
  short default_z = std::numeric_limits<short>::max();

  if (terrain_grid)
    dgc_grid_free(terrain_grid);

  terrain_grid = dgc_grid_initialize( map_resolution, map_width / map_resolution, map_width / map_resolution, sizeof(short), &default_z );
  dgc_grid_recenter_grid(terrain_grid, 0, 0);

  process_laser_scan(vspin, labels);
  bin_points();
  label_obstacle_points();
  calc_local_z();
//  calc_slopes();

//  double start = dgc_get_time();

  int max_scan_width = 0;
  for(int i = 0; i < NUM_LASER_BEAMS; i++) {
    max_scan_width = std::max(max_scan_width, laser_scan[i].num_points);
  }

  BKCutGraph* graph = new BKCutGraph(max_scan_width * NUM_LASER_BEAMS);

  int index = 0;
  for(int i = 0; i < NUM_LASER_BEAMS; i++) {
    int scan_width = laser_scan[i].num_points;
    for(int j = 0; j < scan_width; j++) {
//      int l = rings[i].idx;
      int l = i;
//      if (!laser_scan[l].laser_point[j].valid)
//        continue;

      graph->add_weight(index, BKCutGraph::SOURCE, ground_weights[0]);
      graph->add_weight(index, BKCutGraph::SINK, ground_weights[1]);

      if (laser_scan[l].laser_point[j].obstacle)
        graph->add_weight(index, BKCutGraph::SINK, ground_weights[2]);

//      int r = laser_scan[l].laser_point[j].point->range;
//      int b = ringsIdx[l];
//      if ((r < thresholds[2] * min_ground_range[b]) || (r > max_ground_range))
//        graph->add_weight(index, BKCutGraph::SINK, weights[3]);
//
//      if (laser_scan[l].laser_point[j].local_z > thresholds[1]) {
//        graph->add_weight(index, BKCutGraph::SINK, weights[4]);
//      }

//      if (laser_scan[l].laser_point[j].slope > thresholds[1])
//        graph->add_weight(index, BKCutGraph::SINK, weights[4]); // * laser_scan[l].laser_point[j].slope);
//
//      if (laser_scan[l].laser_point[j].slope > thresholds[2])
//        graph->add_weight(index, BKCutGraph::SINK, weights[5]); // * laser_scan[l].laser_point[j].slope2);

      index++;
    }
  }


//  // add left/right neighbors
//  int x = 0;
//  for(int i = 0; i < NUM_LASER_BEAMS; i++) {
//    for(int j = 1; j < scan_width; j++) {
//      if (!laser_scan[i].laser_point[j].valid)
//        continue;
//      int from = x + j;
//      int to = from - 1;
////      double dist = fabs((laser_scan[i].laser_point[j].point->range - laser_scan[i].laser_point[j-1].point->range) * 0.01);
//
//      float dx = (laser_scan[i].laser_point[j].point->x - laser_scan[i].laser_point[j-1].point->x) * 0.01;
//      float dy = (laser_scan[i].laser_point[j].point->y - laser_scan[i].laser_point[j-1].point->y) * 0.01;
//      float dist = std::min(sqrt(dx*dx + dy*dy), EPSILON);
//      int weight = weights[6];
//      weight += std::min(100.0f, weights[7] / dist);
//      if (dist < thresholds[3]) {
//        weight += weights[8];
//      }
//      if (weight > 0)
//        graph->add_edge(from, to, weight, weight);
//
//    }
//    x += scan_width;
//  }
//
//  // add up/down neighbors
//  x = 0;
//  for(int i = 0; i < NUM_LASER_BEAMS-1; i++) {
//    for(int j = 0; j < scan_width; j++) {
//      if (!laser_scan[i].laser_point[j].valid)
//        continue;
//      int from = x + j;
//      int to = from + scan_width;
//      float dx = (laser_scan[i].laser_point[j].point->x - laser_scan[i+1].laser_point[j].point->x) * 0.01;
//      float dy = (laser_scan[i].laser_point[j].point->y - laser_scan[i+1].laser_point[j].point->y) * 0.01;
//      float dist = std::min(sqrt(dx*dx + dy*dy), EPSILON);
//      int weight = weights[9];
//      weight += std::min(100.0f, weights[10] / dist);
//      if (dist < thresholds[4]) {
//        weight += weights[11];
//      }
//      if (weight > 0)
//        graph->add_edge(from, to, weight, weight);
//
//    }
//    x += scan_width;
//  }

//  double flow = dgc_get_time();
  graph->maxflow();

//  printf("\n");
//  printf("flow: %f\n", dgc_get_time() - flow);

  index = 0;
  for(int i = 0; i < NUM_LASER_BEAMS; i++) {
    int scan_width = laser_scan[i].num_points;
    for(int j = 0; j < scan_width; j++) {
//      if (!laser_scan[i].laser_point[j].valid)
//        continue;
      laser_scan[i].laser_point[j].label->intensity = (graph->what_segment(index) == BKCutGraph::SOURCE) ? LABEL_GROUND : LABEL_OBSTACLE;
      index++;
    }
  }

//  printf("time: %f\n", dgc_get_time() - start);

  delete graph;
}

void classify_points(dgc_velodyne_spin *vspin) {
  initialize_features();

  for(int i = 0; i < vspin->num_scans; i++) {
    for(int j = 0; j < VELO_BEAMS_IN_SCAN; j++) {
      if (vspin->scans[i].p[j].intensity != LABEL_UNKNOWN)
        continue;

      unsigned short r = vspin->scans[i].p[j].range;

      if (r > max_ground_range) {
        vspin->scans[i].p[j].intensity = LABEL_OBSTACLE;
        continue;
      }

      int l = j + vspin->scans[i].block * 32;
      int b = ringsIdx[l];

      if ((r < min_ground_range[b]) || (r > max_ground_range)) {
        vspin->scans[i].p[j].intensity = LABEL_OBSTACLE;
      }
    }
  }
}

void dgc_velodyne_spin_copy( dgc_velodyne_spin &dst, const dgc_velodyne_spin &src )
{
  dst.scans = (dgc_velodyne_scan_p)realloc( dst.scans, src.num_scans *
      sizeof(dgc_velodyne_scan_t));
  dgc_test_alloc(dst.scans);
  memcpy( dst.scans, src.scans, src.num_scans *
      sizeof(dgc_velodyne_scan_t) );
  dst.num_scans = src.num_scans;
}

void calc_features(FILE* fp, int spin, dgc_velodyne_spin *vspin, dgc_velodyne_spin *labels) {
  initialize_features();

  // TODO: update with next_laser_scan
  for (int l=0; l<NUM_LASER_BEAMS; l++) {
    laser_scan[l].num_points = 0;
  }


  for(int i = 0; i < vspin->num_scans; i++) {
    for(int j = 0; j < VELO_BEAMS_IN_SCAN; j++) {
      int l = j + vspin->scans[i].block * 32;
      int e = vspin->scans[i].encoder + rings[ringsIdx[l]].h_offset;
      if (e<0) e += VELO_NUM_TICKS;
      if (e>=VELO_NUM_TICKS) e -= VELO_NUM_TICKS;
      int n = laser_scan[l].num_points;
      if (n<MAX_POINTS_PER_SCAN) {
        laser_scan[l].laser_point[n].scan       = &(vspin->scans[i]);
        laser_scan[l].laser_point[n].point      = &(vspin->scans[i].p[j]);
        laser_scan[l].laser_point[n].encoder    = e;
        laser_scan[l].laser_point[n].obstacle   = labels->scans[i].p[j].intensity;
        laser_scan[l].laser_point[n].valid      = TRUE;
        laser_scan[l].num_points++;
      }
      laser_scan[l].robot = &(vspin->scans[i].robot);
    }
  }
  // *************************************************************
  // * Put beams in bins
  // *************************************************************
  for (int l=0; l<NUM_LASER_BEAMS; l++) {
    for (int i=0; i<720; i++) {
      bins[l][i].num_beams = 0;
    }
  }

  for(int l=0; l<NUM_LASER_BEAMS; l++) {
    for(int i=0; i<laser_scan[l].num_points; i++) {
      if (laser_scan[l].laser_point[i].valid) {
        int e = laser_scan[l].laser_point[i].encoder;
        int b = (int)floor(e / ((float)VELO_NUM_TICKS/720.0f));
        int n = bins[l][b].num_beams;
        if ((e<VELO_BLIND_SPOT_START || e>VELO_BLIND_SPOT_STOP) && n<MAX_BEAMS_IN_BIN) {
          bins[l][b].beam[n] = &laser_scan[l].laser_point[i];
          bins[l][b].beam[n]->encoder = e;
          bins[l][b].num_beams++;
        }
      }
    }
  }


  for (int l = 0; l < NUM_LASER_BEAMS; l++) {
    for (int i = 0; i < laser_scan[l].num_points; i++) {
      if (laser_scan[l].laser_point[i].obstacle == LABEL_UNKNOWN)
        continue;

      // beam, encoder
      fprintf(fp, "%d", spin);
      fprintf(fp, " %d", ringsIdx[l]);
      fprintf(fp, " %u", laser_scan[l].laser_point[i].encoder);

      // raw values
      fprintf(fp, " %d", laser_scan[l].laser_point[i].point->x);
      fprintf(fp, " %d", laser_scan[l].laser_point[i].point->y);
      fprintf(fp, " %d", laser_scan[l].laser_point[i].point->z);
      fprintf(fp, " %u", laser_scan[l].laser_point[i].point->range);
      fprintf(fp, " %u", laser_scan[l].laser_point[i].point->intensity);

      // bin stats bins
      int e = laser_scan[l].laser_point[i].encoder;
      int b = (int)floor(e / ((float)VELO_NUM_TICKS/720.0f));
      int r = (int)laser_scan[l].laser_point[i].point->range;

      short max_z = 0;
      for(int m=0; m<bins[l][b].num_beams; m++) {
        short delta_z = (laser_scan[l].laser_point[i].point->z - bins[l][b].beam[m]->point->z);
        if (delta_z > max_z)
          max_z = delta_z;
      }
      fprintf(fp, " %d", max_z);

      for (int n = 1; n < 5; n++) {
        int l1 = ringsIdx[l] - n;
        if (l1 >= 0) {
          l1 = rings[l1].idx;
          int min_range = 0;
          for(int m=0; m<bins[l1][b].num_beams; m++) {
            int delta_range = (r - (int)bins[l1][b].beam[m]->point->range);
            if (delta_range < min_range)
              min_range = delta_range;
          }
          fprintf(fp, " %d", min_range);
        } else {
          fprintf(fp, " %d", -1);
        }

        l1 = ringsIdx[l] + n;
        if (l1 < NUM_LASER_BEAMS) {
          l1 = rings[l1].idx;
          int min_range = 0;
          for(int m=0; m<bins[l1][b].num_beams; m++) {
            int delta_range = ((int)bins[l1][b].beam[m]->point->range - r);
            if (delta_range < min_range)
              min_range = delta_range;
          }
          fprintf(fp, " %d", min_range);
        } else {
          fprintf(fp, " %d", -1);
        }
      }

      // class
      fprintf(fp, " %u\n", laser_scan[l].laser_point[i].obstacle);
    }
  }
}

void evaluate_labels(dgc_velodyne_spin& gt, dgc_velodyne_spin& labels, int* tp, int* fp, int* tn, int*fn) {
  for (int i = 0; i < gt.num_scans; i++) {
    for (int j = 0; j < VELO_BEAMS_IN_SCAN; j++) {
      switch (gt.scans[i].p[j].intensity) {
      case(LABEL_UNKNOWN):
          break;

      case(LABEL_GROUND):
          if (labels.scans[i].p[j].intensity == LABEL_GROUND)
            (*tn)++;
          else
            (*fp)++;
          break;

      case(LABEL_OBSTACLE):
          if (labels.scans[i].p[j].intensity == LABEL_OBSTACLE)
            (*tp)++;
          else
            (*fn)++;
          break;


      case(LABEL_NOISE):
      default:
          break;
      }
    }
  }
}

// 1, 335, 804, 948, 1225

int train_frames[] = { 1, 335, 804, 948, 1225 };
int num_train_frames = 5;
float test_segmentation(int scenes, float* fp_rate, float* fn_rate) {
  const int num_scenes = 1600;
  static int good_scene[1600];
  static bool init = false;
  static dgc_velodyne_spin train_spins[5];
  static dgc_velodyne_spin test_spins[5];
  static dgc_velodyne_spin spins[5];

  double lat,lon,alt;

  if (!init) {
    init = true;

    for (int i=0; i<num_train_frames; i++) {
      strcpy(labels_filename, vlf_filename);
      sprintf(labels_ext, ".%06d", train_frames[i]);
      strcat(labels_filename, labels_ext);
      train_spins[i].load(labels_filename, &lat, &lon, &alt);
      spins[i].load(velodyne_file, velodyne_config, &velodyne_index, train_frames[i], &lat, &lon, &alt);
    }
    dgc_velodyne_spin_copy(test_spin, spins[2]);

    for (int i=0; i<num_scenes; i++)
      good_scene[i] = true;
  }

  int tp = 0;
  int fp = 0;
  int tn = 0;
  int fn = 0;

  if (scenes == 1) {
    dgc_velodyne_spin_copy(test_spin, spin);
    if (test_spin.num_scans != spin.num_scans)
      printf("WARNING: scans in spins don't match!!\n");
    segment_points(&spin, &test_spin);
    evaluate_labels(labels_spin, test_spin, &tp, &fp, &tn, &fn);
    printf("tp: %d\t fp: %d\t tn:%d\t fn:%d\n", tp, fp, tn, fn);
  } else {
    for (int s=0; s < num_train_frames; s++) {
      dgc_velodyne_spin_copy(test_spin, spins[s]);
      if (test_spin.num_scans != spins[s].num_scans) {
        printf("#WARNING: scans in spins don't match!!\n");
        continue;
      }
      segment_points(&spins[s], &test_spin);
      evaluate_labels(train_spins[s], test_spin, &tp, &fp, &tn, &fn);
      printf("tp: %d\t fp: %d\t tn:%d\t fn:%d\n", tp, fp, tn, fn);
    }
//    int evals = 0;
//    for (int s=1; ((s < num_scenes) && (evals < scenes+1)); s++) {
//      if (!good_scene[s])
//        continue;
//
//      strcpy(labels_filename, vlf_filename);
//      sprintf(labels_ext, ".%06d", s);
//      strcat(labels_filename, labels_ext);
//      if (labels_spin.load(labels_filename, &lat, &lon, &alt) == 0) {
//        evals++;
//        spin.load(velodyne_file, velodyne_config, &velodyne_index, s, &lat, &lon, &alt);
//
//        segment_points(&spin, &test_spin);
//        evaluate_labels(labels_spin, test_spin, &tp, &fp, &tn, &fn);
//  //      printf("s:%d\n", s);
//        printf("tp: %d\t fp: %d\t tn:%d\t fn:%d\n", tp, fp, tn, fn);
//  //      printf("fp rate: %f\t fn rate: %f\n", (float)fp/(fp+tp), (float)fn/(fn+tn));
//      } else {
//        good_scene[s] = false;
//      }
//    }
  }

  float accuracy = (float)(tp + tn) / (tp + tn + fp + fn);
  float score = (float)(tp + tn) / (tp + tn + 100.0 * fp + fn);
  printf("fp rate: %f\t fn rate: %f\t accuracy: %f\t score: %f\n", (float)fp/(fp+tn), (float)fn/(fn+tp), accuracy, score);

//  printf("background rate:  %f\n", (float)(tp + fn) / (tp + tn + fp + fn) );

  *fp_rate = (float)fp/(fp+tp);
  *fn_rate = (float)fn/(fn+tn);

  return score;
}

// unsigned int weights[] = {1, 0, 10, 10, 0, 0, 1, 1 };
// float thresholds[] = { 0.0002, 0.2, 0.8 };

void train_weight(int w, int scenes) {
  unsigned best_weight = 0;
//  float best_fp = 1.0;
//  float best_fn = 1.0;

  float best = 0;
  for (unsigned int i=0; i<15; i++) {
    float fp, fn;
    ground_weights[w] = i;
    float score = test_segmentation(scenes, &fp, &fn);

//    if ((fp < best_fp) && (best_fn < fn)) {
//      best_fp = fp;
//      best_fn = fn;
//      best_weight = weights[w];
//    }

    if (score > best) {
      best = score;
      best_weight = ground_weights[w];
    }
  }

  ground_weights[w] = best_weight;
  printf("best %d weight: %d\n", w, ground_weights[w]);
  fflush(stdout);
}

void train_threshold(int t, int scenes) {

  double     stepsize = 10.0;

  float fp = 1.0;
  float fn = 1.0;
  float best = test_segmentation(scenes, &fp, &fn);

  while (stepsize>EPSILON) {
    ground_thresholds[t] += stepsize;
    float score_plus = test_segmentation(scenes, &fp, &fn);

    ground_thresholds[t] -= stepsize;
    ground_thresholds[t] -= stepsize;
    float score_minus = test_segmentation(scenes, &fp, &fn);

    ground_thresholds[t] += stepsize;

    if (score_plus > best) {
      ground_thresholds[t] += stepsize;
      best = score_plus;
    } else if (score_minus > best) {
      ground_thresholds[t] -= stepsize;
      best = score_minus;
    } else {
      stepsize /= 2.0;
    }
  }
  printf("best %d threshold: %f\n", t, ground_thresholds[t]);
  fflush(stdout);
}

void rand_weights() {
  printf("rand_weights: ");
  for (int i=0; i < NUM_GROUND_WEIGHTS; i++) {
    ground_weights[i] = ( (float)rand() / RAND_MAX) * 100;
    printf("%d\t", ground_weights[i]);
  }
  printf("\n");
}

void rand_thresholds() {
  for (int i=1; i < NUM_GROUND_THRESHOLDS; i++)
    ground_thresholds[i] = rand();
}

void train_segmentation() {
//  train_weight(0,1);

//  rand_weights();
//  rand_thresholds();

//  for (int i = 0; i < NUM_WEIGHTS; i++)
//    train_weight(NUM_WEIGHTS - i - 1, 1);

  for (int i=0; i<NUM_GROUND_THRESHOLDS; i++)
    train_threshold(i, 1);

//  for (int i = 0; i < NUM_WEIGHTS; i++)
//    train_weight(i, 1);
//
//  for (int i=0; i<NUM_THRESHOLDS; i++)
//    train_threshold(i, 10);
//
//  for (int i = 0; i < NUM_WEIGHTS; i++)
//    train_weight(i, 10);

  printf("-- unsigned int weights[] = {");
  for (int i = 0; i< NUM_GROUND_WEIGHTS; i++)
    printf("%d, ", ground_weights[i]);

  printf("}\n-- float thresholds[] = { ");
  for (int i = 0; i<NUM_GROUND_THRESHOLDS; i++)
    printf("%f,  ", ground_thresholds[i]);
  printf("}\n");
  fflush(stdout);

  for (int i=0; i<NUM_GROUND_THRESHOLDS; i++)
    train_threshold(i, 2);
//
//  for (int i = 0; i < NUM_WEIGHTS; i++)
//    train_weight(i, 100);

  printf("-- unsigned int weights[] = {");
  for (int i = 0; i< NUM_GROUND_WEIGHTS; i++)
    printf("%d, ", ground_weights[i]);

  printf("}\n-- float thresholds[] = { ");
  for (int i = 0; i<NUM_GROUND_THRESHOLDS; i++)
    printf("%f,  ", ground_thresholds[i]);
  printf("}\n");

  float fp, fn;
  printf("score: %f\n", test_segmentation(2, &fp, &fn));
  fflush(stdout);

//  for (int i=1; i<NUM_THRESHOLDS; i++)
//    train_threshold(i, 5);
//
//  for (int i = 0; i < NUM_WEIGHTS; i++)
//    train_weight(i, 5);
//
//  printf("-- unsigned int weights[] = {");
//  for (int i = 0; i< NUM_WEIGHTS; i++)
//    printf("%d, ", weights[i]);
//
//  printf("}\n-- float thresholds[] = { ");
//  for (int i = 0; i<NUM_THRESHOLDS; i++)
//    printf("%f,  ", thresholds[i]);
//  printf("}\n");
////
//  for (int i=0; i<NUM_THRESHOLDS; i++)
//    train_threshold(i, 5);
//
//  for (int i = 0; i < NUM_WEIGHTS; i++)
//    train_weight(i, 20);
//
//  for (int i=0; i<NUM_THRESHOLDS; i++)
//    train_threshold(i, 20);
//
//  printf("-- unsigned int weights[] = {");
//  for (int i = 0; i< NUM_WEIGHTS; i++)
//    printf("%d, ", weights[i]);
//
//
//  printf("}\n-- float thresholds[] = { ");
//  for (int i = 0; i<NUM_THRESHOLDS; i++)
//    printf("%f,  ", thresholds[i]);
//  printf("}\n");
//
//
//  for (int i=0; i<NUM_THRESHOLDS; i++)
//    train_threshold(i, 50);
//
//
//  for (int i = 0; i < NUM_WEIGHTS; i++)
//    train_weight(i, 100);
//
//  for (int i = 0; i < NUM_WEIGHTS; i++)
//    train_weight(i, 100);
//
//  printf("-- unsigned int weights[] = {");
//  for (int i = 0; i< NUM_WEIGHTS; i++)
//    printf("%d, ", weights[i]);
//
//  printf("}\n-- float thresholds[] = { ");
//  for (int i = 0; i<NUM_THRESHOLDS; i++)
//    printf("%f,  ", thresholds[i]);
//  printf("}\n");
}

void calc_features() {
  double lat,lon,alt;
  strcpy(features_filename, vlf_filename);
  strcat(features_filename, ".features");

  FILE* fp = fopen(features_filename, "w");
//  {
//    int s = 0;
  for (int s=0; s < 1500; s++) {
    strcpy(labels_filename, vlf_filename);
    sprintf(labels_ext, ".%06d", s);
    strcat(labels_filename, labels_ext);
    if (labels_spin.load(labels_filename, &lat, &lon, &alt) == 0) {
      spin.load(velodyne_file, velodyne_config, &velodyne_index, s, &lat, &lon, &alt);

      calc_features(fp, s, &spin, &labels_spin);
    }
  }
  fclose(fp);

  spin.load(velodyne_file, velodyne_config, &velodyne_index, current_spin_num, &lat, &lon, &alt);
  strcpy(labels_filename, vlf_filename);
  sprintf(labels_ext, ".%06d", current_spin_num);
  strcat(labels_filename, labels_ext);
  labels_spin.load(labels_filename, &lat, &lon, &alt);
}

void load_labels() {
  double lat, lon, alt;
  strcpy(labels_filename, vlf_filename);
  sprintf(labels_ext, ".%06d", current_spin_num);
  strcat(labels_filename, labels_ext);
  printf("loading %s\n", labels_filename);
  if (labels_spin.load(labels_filename, &lat, &lon, &alt) != 0) {
    printf("could not load %s\n", labels_filename);
    dgc_velodyne_spin_copy(labels_spin,spin);
//    labels_spin.copy(spin);
    label_all_points(&labels_spin, LABEL_UNKNOWN);
  } else if (labels_spin.num_scans != spin.num_scans) {
    printf("#WARNING: label scans(%d) != spin scans (%d)\n", labels_spin.num_scans, spin.num_scans);
    dgc_velodyne_spin_copy(labels_spin,spin);
    label_all_points(&labels_spin, LABEL_UNKNOWN);
  }
}

void save_labels() {
  if (labelled) {
    labels_spin.save(labels_filename, 0, 0, 0);
    labelled = false;
  }
}

void keyboard(unsigned char key, int x, int y)
{
  double applanix_lat, applanix_lon, applanix_alt, scene_x, scene_y;
  double smooth_x, smooth_y;
  int delta = 0;

  /* figure out where the user clicked on the plane */
  gui3D_pick_point(x, y, &scene_x, &scene_y);
  smooth_x = scene_x;
  smooth_y = scene_y;

  smooth_x += velodyne_index.spin[current_spin_num].pose[0].smooth_x;
  smooth_y += velodyne_index.spin[current_spin_num].pose[0].smooth_y;

  if(key >= '1' && key <= '9')
    delta = key - '0';
  else
    switch(key) {
    case 'i': case 'I':
      dgc_imagery_cycle_imagery_type();
      break;
    case 27: case 'q': case 'Q':
      save_labels();
      exit(0);
      break;

    case 'c':
      color_mode++;
      if (color_mode > 4)
        color_mode = 0;
      break;

    case 'd':
      show_distance = !show_distance;
      break;

    case '+':
      ground_threshold += 0.01;
      break;
    case '-':
      ground_threshold -= 0.01;
      break;
    case 'x':
      show_clusters = 1 - show_clusters;
//      if (show_clusters)
//        generate_clusters(current_spin_num, spin.num_scans, spin.scans, velodyne_config, ground_threshold - DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT, clusters);
      break;
    case 'X':
//      if (show_clusters)
//        generate_clusters(current_spin_num, spin.num_scans, spin.scans, velodyne_config, ground_threshold - DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT, clusters);
      break;

    case 'T':
        train_segmentation();
      break;

    case 'R':
      segment_points_badly(&spin, &labels_spin);
      break;

    case 'r':
      rand_weights();
      break;

    case 't':
        float fn, fp;
        test_segmentation(1, &fp, &fn);
//        printf("fp rate: %f\t fn rate: %f\t accuracy: %f\n", fp, fn, accuracy);
//      test_clusters(spin.num_scans, spin.scans, velodyne_config, ground_threshold - DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT, clusters, scene_x, scene_y);
      break;

    case 'l':
      show_spin = 1 - show_spin;
      break;

    case 'L':
      segment_points(&spin, &labels_spin);
      break;

    case 'g':
      delta = 1225 - current_spin_num;
//      delta = 335 - current_spin_num;
      show_grid = 1 - show_grid;
      break;

    case 'a':
      current_label = LABEL_NOISE;
      break;
    case 'o':
      current_label = LABEL_UNKNOWN;
      break;
    case 'e':
      current_label = LABEL_GROUND;
      break;
    case 'u':
      current_label = LABEL_OBSTACLE;
      break;

    case 'F':
      calc_features();
      break;

    case 's':
      save_labels();
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

    case 'f':
      draw_flat = !draw_flat;
      break;

    default:
      break;
    }

  if(delta) {
    save_labels();

    current_spin_num += delta;
    if(current_spin_num >= velodyne_index.num_spins)
      current_spin_num = velodyne_index.num_spins - 1;
    if(current_spin_num < 0)
      current_spin_num = 0;
    spin.load(velodyne_file, velodyne_config, &velodyne_index, current_spin_num,
        &applanix_lat, &applanix_lon, &applanix_alt);

    load_labels();
    if (laser_scan != NULL)
      process_laser_scan(&spin, &labels_spin);

//    if (show_clusters) {
//      spin.scans->counter = current_spin_num;
//      generate_clusters(current_spin_num, spin.num_scans, spin.scans, velodyne_config, ground_threshold - DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT, clusters);
//    }
  }

  gui3D_forceRedraw();
}

void mouse(int button, int state, int x, int y)
{
  double scene_x, scene_y;

  /* figure out where the user clicked on the plane */
  gui3D_pick_point(x, y, &scene_x, &scene_y);
  scene_x += velodyne_index.spin[current_spin_num].pose[0].smooth_x;
  scene_y += velodyne_index.spin[current_spin_num].pose[0].smooth_y;

  if (gui3D.modifiers & GLUT_ACTIVE_CTRL) {
    if (button == GLUT_LEFT_BUTTON) {
      if (state == GLUT_DOWN) {
        rect_x = x;
        rect_y = gui3D.window_height - y;
        rect_x2 = rect_x;
        rect_y2 = rect_y;
        show_rect = 1;
      } else if (state == GLUT_UP) {
        if (show_rect) {
          show_rect = 0;
          double robot_smooth_x = velodyne_index.spin[current_spin_num].pose[0].smooth_x;
          double robot_smooth_y = velodyne_index.spin[current_spin_num].pose[0].smooth_y;
          double robot_smooth_z = velodyne_index.spin[current_spin_num].pose[0].smooth_z;
          label_selected_points(&spin, &labels_spin, current_label, robot_smooth_x, robot_smooth_y, robot_smooth_z - DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT, draw_flat);
          labelled = true;
        }
      }

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

void motion(int x, int y)
{
  double scene_x, scene_y;

  /* figure out where the user clicked on the plane */
  gui3D_pick_point(x, y, &scene_x, &scene_y);
  scene_x += velodyne_index.spin[current_spin_num].pose[0].smooth_x;
  scene_y += velodyne_index.spin[current_spin_num].pose[0].smooth_y;

  rect_x2 = x;
  rect_y2 = gui3D.window_height - y;

  last_mouse_x = x;
  last_mouse_y = y;
}

void draw_rect()
{
  if (show_rect)
  {
    glEnable(GL_BLEND);
    int color = (current_label % 19);
    glColor4f(colors[color][0]/255.0, colors[color][1]/255.0, colors[color][2]/255.0, .4);
    glBegin(GL_POLYGON);
    glVertex2d(rect_x2, rect_y2);
    glVertex2d(rect_x2, rect_y);
    glVertex2d(rect_x, rect_y);
    glVertex2d(rect_x, rect_y2);
    glEnd();
    glDisable(GL_BLEND);
  }
}

void draw_clusters(dgc_pose_t pose, double origin_x, double origin_y,
    double origin_z, int flat)
{

  if(flat)
    glDisable(GL_DEPTH_TEST);

  glPushMatrix();

//  glTranslatef(pose.x, pose.y, pose.z);
//  glScalef(0.01, 0.01, 0.01);

  glBegin(GL_POINTS);
  int num_clusters = clusters.size();
  for (int i=0; i<num_clusters; i++) {

    int color = (i % 19);
    glColor3f(colors[color][0]/255.0, colors[color][1]/255.0, colors[color][2]/255.0);

    laser_cluster_t* c = clusters[i];
    int count = c->size();
    for (int n=0; n < count; n++) {
      dgc_velodyne_point_p p = c->at(n);
      if (p == NULL)
        continue;
      double x = p->x * 0.01 + pose.x;
      double y = p->y * 0.01 + pose.y;
      double z = p->z * 0.01 + pose.z;

      glVertex3f(x - origin_x, y - origin_y, z - origin_z);
    }
  }
  glEnd();

  glPopMatrix();
}

void draw_grid(dgc_grid_p grid)
{
  glPushMatrix();
  glTranslatef(0, 0, DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT);

  glBegin(GL_QUADS);

  double resolution = grid->resolution;
  double dr = resolution / 2.0;

  glColor4f(0.0, 0.0, 0.0, 0.5);
  for (int c=0; c<grid->cols; c++) {
    for (int r=0; r<grid->rows; r++) {
      double x, y;
      dgc_grid_rc_local_to_xy(grid, r, c, &x, &y);
      float z = *((short*)dgc_grid_get_rc_local(grid, r, c)) * 0.01;
      glVertex3f(x-dr, y-dr, z);
      glVertex3f(x-dr, y+dr, z);
      glVertex3f(x+dr, y+dr, z);
      glVertex3f(x+dr, y-dr, z);
    }
  }

  glEnd();
  glPopMatrix();
}

void draw_laser_scan(double origin_x, double origin_y, double origin_z, int flat)
{
  if(flat)
    glDisable(GL_DEPTH_TEST);

  glBegin(GL_POINTS);
  for (int i = 0; i < NUM_LASER_BEAMS; i++) {
    for (int j = 0; j < laser_scan[i].num_points; j++) {
      double x = laser_scan[i].laser_point[j].point->x * 0.01 + laser_scan[i].robot->x;
      double y = laser_scan[i].laser_point[j].point->y * 0.01 + laser_scan[i].robot->y;
      double z = laser_scan[i].laser_point[j].point->z * 0.01 + laser_scan[i].robot->z;

      if (z - origin_z < ground_threshold)
        continue;

      double in;
      int b, color, en;
      switch (color_mode) {
        default:
        case 1:
          in = laser_scan[i].laser_point[j].point->intensity / 255.0;
          glColor3f(in, in, in);
          break;

        case 2:
          in = (double)laser_scan[i].laser_point[j].encoder / (double)VELO_NUM_TICKS;
          glColor3f(in, 1-in, 0);
          break;

        case 3:
          en = (laser_scan[i].laser_point[j].scan->encoder + 18000) % 36000;
          in = en / (double)VELO_NUM_TICKS;
          glColor3f(in, 1-in, 0);
          break;

        case 4:
          b = (int)floor(laser_scan[i].laser_point[j].encoder / ((float)VELO_NUM_TICKS/(float)BINS_PER_REV));
          color = b % 16;
          glColor3f(colors[color][0]/255.0, colors[color][1]/255.0, colors[color][2]/255.0);
          break;
      }

      if(flat)
        glVertex3f(x - origin_x, y - origin_y, 0);
      else
        glVertex3f(x - origin_x, y - origin_y, z - origin_z);
    }
  }
  glEnd();

  if(flat)
    glEnable(GL_DEPTH_TEST);
}

void draw_spin(dgc_velodyne_spin *vspin, dgc_velodyne_spin *labels, dgc_velodyne_spin *guess,
    double origin_x, double origin_y, double origin_z, int flat)
{
  int i, j;
  double x, y, z;

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

      if (z - origin_z < ground_threshold)
        continue;

      if (labels && guess) {

        if (labels->scans[i].p[j].intensity == LABEL_UNKNOWN) {
          double in = vspin->scans[i].p[j].intensity / 255.0;
          glColor3f(in, in, in);
        } else if (labels->scans[i].p[j].intensity == guess->scans[i].p[j].intensity) {
          int color = (labels->scans[i].p[j].intensity % 19);
          glColor3f(colors[color][0]/255.0, colors[color][1]/255.0, colors[color][2]/255.0);
        } else if (guess->scans[i].p[j].intensity == LABEL_GROUND) {
          int color = 4;
          glColor3f(colors[color][0]/255.0, colors[color][1]/255.0, colors[color][2]/255.0);
        } else if (guess->scans[i].p[j].intensity == LABEL_OBSTACLE) {
          int color = 5;
          glColor3f(colors[color][0]/255.0, colors[color][1]/255.0, colors[color][2]/255.0);
        }
      } else {


        if (labels && (labels->scans[i].p[j].intensity != LABEL_UNKNOWN)) {
          int color = (labels->scans[i].p[j].intensity % 19);
          glColor3f(colors[color][0]/255.0, colors[color][1]/255.0, colors[color][2]/255.0);
        } else {
          int e, b, l, l0, l1, color = 0;
          double in;
          switch (color_mode) {
            case 0:
              color = j % 16;
              glColor3f(colors[color][0]/255.0, colors[color][1]/255.0, colors[color][2]/255.0);
              break;

            case 1:
              l = j + vspin->scans[i].block * 32;
              e = vspin->scans[i].encoder + rings[ringsIdx[l]].h_offset;
//              int e = laser_scan[l].laser_point[i].encoder;
              b = (int)floor(e / ((float)VELO_NUM_TICKS/(float)BINS_PER_REV));
              color = b % 16;

              glColor3f(colors[color][0]/255.0, colors[color][1]/255.0, colors[color][2]/255.0);
              break;

            case 2:

              in = (float)((vspin->scans[i].encoder + VELO_NUM_TICKS / 2) % VELO_NUM_TICKS)  / (float)VELO_NUM_TICKS;
              glColor3f(in, 0, 0);
              break;

            case 3:
              l = j + vspin->scans[i].block * 32;
              e = vspin->scans[i].encoder + rings[ringsIdx[l]].h_offset;
              b = (int)floor(e / ((float)VELO_NUM_TICKS/(float)BINS_PER_REV));
              l1 = rings[l].idx;            /* shorter beam */
              l0 = rings[rings[l].pb].idx;   /* longer beam */
              if ((bins[l0][b].encoder < VELO_SPIN_START) != (bins[l1][b].encoder < VELO_SPIN_START)) {
                color = 0;
                glColor3f(colors[color][0]/255.0, colors[color][1]/255.0, colors[color][2]/255.0);
              } else {
                in = vspin->scans[i].p[j].intensity / 255.0;
                glColor3f(in, in, in);
              }

              break;

            case 4:
              l = j + vspin->scans[i].block * 32;
              e = vspin->scans[i].encoder + rings[ringsIdx[l]].h_offset;
              if ((e > VELO_BLIND_SPOT_START) && (e < VELO_BLIND_SPOT_STOP)) {
                color = 0;
                glColor3f(colors[color][0]/255.0, colors[color][1]/255.0, colors[color][2]/255.0);
              } else {
                in = vspin->scans[i].p[j].intensity / 255.0;
                glColor3f(in, in, in);
              }

              break;
          }

//          int color = j % 16;
//          glColor3f(colors[color][0]/255.0, colors[color][1]/255.0, colors[color][2]/255.0);
//          double in = vspin->scans[i].p[j].intensity / 255.0;
//          glColor3f(in, in, in);
        }
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
//  draw_path(robot_smooth_x, robot_smooth_y);

  glDisable(GL_DEPTH_TEST);

//  for(i = 0; i < carlist.num_cars(); i++)
//    carlist.car[i].draw(robot_smooth_x, robot_smooth_y, current_spin_num);

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

  if (show_distance) {
    // draw distance rings
    glTranslatef(0,0,-0.02);
    for(int i = 10; i <= 120; i += 10) {
      if (i%50) {
        glColor3f(0.5, 0.5, 0.5);
      } else {
        glColor3f(1.0, 0.5, 0.5);
      }
      glBegin(GL_LINE_LOOP);
      for(int j = 0; j < 100; j++) {
        double angle = j / 100.0 * M_PI * 2;
        glVertex3f(i * cos(angle), i * sin(angle), 0);
      }
      glEnd();
    }
  }

  glEnable(GL_DEPTH_TEST);

  /* draw the velodyne spins */
  glColor3f(1, 1, 1);
  if(large_points)
    glPointSize(3);
  else
    glPointSize(1);
  dgc_transform_identity(t);

  if (show_clusters)
    draw_clusters(spin.scans[0].robot, robot_smooth_x, robot_smooth_y, robot_smooth_z -
        DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT, draw_flat);
  else {
    if (show_spin)
      draw_spin(&spin, &labels_spin, NULL, robot_smooth_x, robot_smooth_y, robot_smooth_z -
          DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT, draw_flat);
    else
      draw_laser_scan(robot_smooth_x, robot_smooth_y, robot_smooth_z - DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT, draw_flat);
  }

  if (show_grid && terrain_grid)
    draw_grid(terrain_grid);

  /* draw the passat */
//  glEnable(GL_LIGHTING);
//  glPushMatrix();
//  glRotatef(dgc_r2d(robot_yaw), 0, 0, 1);
//  glRotatef(dgc_r2d(robot_pitch), 0, 1, 0);
//  glRotatef(dgc_r2d(robot_roll), 1, 0, 0);
//  glTranslatef(1.65, 0, -0.6 + DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT);
//  passatwagonmodel_draw(passat, 0, 0, 0);
//  glPopMatrix();
//  glDisable(GL_LIGHTING);
//  glEnable(GL_BLEND);

  /* go to 2D */
  set_display_mode_2D(gui3D.window_width, gui3D.window_height);

  /* draw the info box */
  draw_info_box();

  draw_rect();
}

void timer(int)
{
  if(dgc_imagery_update())
    gui3D_forceRedraw();
  gui3D_add_timerFunc(100, timer, 0);
}

void read_parameters(ParamInterface *pint, int argc, char **argv)
{
//  Param params[] = {
//      {"rndf",      "rndf_file",    DGC_PARAM_FILENAME,    &rndf_filename, 0, NULL },
//
//      {"perception", "hz",   DGC_PARAM_DOUBLE,    &settings.rate_in_hz,  1, NULL },
//      {"perception", "max_sensor_delay",   DGC_PARAM_DOUBLE,    &settings.max_sensor_delay,  1, NULL },
//      {"perception", "clear_sensor_data",   DGC_PARAM_ONOFF,    &settings.clear_sensor_data,  1, NULL },
//      {"perception", "extract_dynamic",   DGC_PARAM_ONOFF,     &settings.extract_dynamic,       1, NULL},
//      {"perception", "overpass_height",   DGC_PARAM_DOUBLE,     &settings.overpass_height,      1, NULL},
//
//      {"perception", "map_resolution",   DGC_PARAM_DOUBLE,    &settings.map_resolution,  0, NULL },
//      {"perception", "map_size_x",   DGC_PARAM_DOUBLE,    &settings.map_size_x,  0, NULL },
//      {"perception", "map_size_y",   DGC_PARAM_DOUBLE,    &settings.map_size_y,  0, NULL },
//      {"perception", "map_cell_threshold",   DGC_PARAM_DOUBLE,    &settings.map_cell_threshold,  1, NULL },
//      {"perception", "map_cell_increase",   DGC_PARAM_INT,    &settings.map_cell_increase,  1, NULL },
//      {"perception", "map_ray_tracing" , DGC_PARAM_ONOFF,     &settings.map_ray_tracing,      1, NULL},
//
//      // {"perception", "gls_output" , DGC_PARAM_ONOFF,     &settings.gls_output,      1, NULL},
//      {"perception", "show_virtual_scan" , DGC_PARAM_ONOFF,    &settings.show_virtual_scan,      1, NULL},
//      {"perception", "show_ray_tracing" , DGC_PARAM_ONOFF,     &settings.show_ray_tracing,      1, NULL},
//
//      {"perception", "use_velodyne" , DGC_PARAM_ONOFF,     &settings.use_velodyne,      1, NULL},
//      {"perception", "velodyne_threshold_factor", DGC_PARAM_DOUBLE,   &settings.velodyne_threshold_factor, 1, NULL },
//      {"perception", "velodyne_max_range", DGC_PARAM_DOUBLE,   &settings.velodyne_max_range, 1, NULL },
//      {"perception", "velodyne_min_beam_diff", DGC_PARAM_DOUBLE,   &settings.velodyne_min_beam_diff, 0, NULL },
//      {"perception", "velodyne_sync",   DGC_PARAM_ONOFF,    &settings.velodyne_sync,  1, NULL },
//
//      {"perception", "stop_zone_dist_before_line" , DGC_PARAM_DOUBLE,  &stop_zones.dist_before_line, 1, NULL},
//      {"perception", "stop_zone_dist_behind_line" , DGC_PARAM_DOUBLE,  &stop_zones.dist_behind_line, 1, NULL},
//      {"perception", "stop_zone_width" , DGC_PARAM_DOUBLE,   &stop_zones.width,  1, NULL},
//      {"perception", "stop_zone_side_shift" , DGC_PARAM_DOUBLE,   &stop_zones.side_shift,  1, NULL},
//      {"perception", "stop_zone_min_height" , DGC_PARAM_DOUBLE,   &stop_zones.min_height,  1, NULL},
//      {"perception", "stop_zone_detect_distance" , DGC_PARAM_DOUBLE,   &stop_zones.detect_distance,  1, NULL},
//
//      { "segmentation", "min_points", DGC_PARAM_INT, &settings.segmentation_settings.min_points, 1, NULL },
//      { "segmentation", "max_points", DGC_PARAM_INT, &settings.segmentation_settings.max_points, 1, NULL },
//      { "segmentation", "min_height", DGC_PARAM_DOUBLE, &settings.segmentation_settings.min_height, 1, NULL },
//      { "segmentation", "kernel_size", DGC_PARAM_INT, &settings.segmentation_settings.kernel_size, 1, NULL },
//      // { "segmentation", "gls_output", DGC_PARAM_ONOFF, &settings.segmentation_settings.gls_output, 1, NULL },
//
//      { "tracker", "min_car_width", DGC_PARAM_INT, &settings.tracker_settings.min_car_width, 1, NULL },
//      { "tracker", "min_car_length", DGC_PARAM_INT, &settings.tracker_settings.min_car_length, 1, NULL },
//      { "tracker", "filter_rndf_max_distance", DGC_PARAM_DOUBLE, &settings.tracker_settings.filter_rndf_max_distance, 1, NULL },
//      { "tracker", "filter_rndf_max_pedestrian_distance", DGC_PARAM_DOUBLE, &settings.tracker_settings.filter_rndf_max_pedestrian_distance, 1, NULL },
//      { "tracker", "merge_distance", DGC_PARAM_DOUBLE, &settings.tracker_settings.merge_dist, 1, NULL },
//      { "tracker", "lateral_merge_dist", DGC_PARAM_DOUBLE, &settings.tracker_settings.lateral_merge_dist, 1, NULL },
//
//      { "tracker", "default_loc_stddev", DGC_PARAM_DOUBLE, &settings.kf_settings.default_loc_stddev, 1, NULL },
//      { "tracker", "default_vel_stddev", DGC_PARAM_DOUBLE, &settings.kf_settings.default_vel_stddev, 1, NULL },
//      { "tracker", "transition_stddev", DGC_PARAM_DOUBLE, &settings.kf_settings.transition_stddev, 1, NULL },
//      { "tracker", "velodyne_stddev", DGC_PARAM_DOUBLE, &settings.kf_settings.velodyne_stddev, 1, NULL },
//      // { "tracker", "velodyne_max_range", DGC_PARAM_DOUBLE, &settings.kf_settings.velodyne_max_range, 1, NULL },
//      { "tracker", "radar_stddev", DGC_PARAM_DOUBLE, &settings.kf_settings.radar_stddev, 1, NULL },
//      { "tracker", "max_dist_correspondence", DGC_PARAM_DOUBLE, &settings.kf_settings.max_dist_correspondence, 1, NULL },
//      { "tracker", "confidence_increment_obs", DGC_PARAM_DOUBLE, &settings.kf_settings.confidence_increment_obs, 1, NULL },
//      { "tracker", "confidence_increment_unobs", DGC_PARAM_DOUBLE, &settings.kf_settings.confidence_increment_unobs, 1, NULL },
//
//      { "tracker", "confidence_decay", DGC_PARAM_DOUBLE, &settings.kf_settings.confidence_decay, 1, NULL },
//      { "tracker", "confidence_max", DGC_PARAM_DOUBLE, &settings.kf_settings.confidence_max, 1, NULL },
//      { "tracker", "confidence_min", DGC_PARAM_DOUBLE, &settings.kf_settings.confidence_min, 1, NULL },
//      { "tracker", "confidence_initial_min", DGC_PARAM_DOUBLE, &settings.kf_settings.confidence_initial_min, 1, NULL },
//      { "tracker", "confidence_initial_max", DGC_PARAM_DOUBLE, &settings.kf_settings.confidence_initial_max, 1, NULL },
//      { "tracker", "confidence_track_min", DGC_PARAM_DOUBLE, &settings.kf_settings.confidence_track_min, 1, NULL },
//      { "tracker", "confidence_publish_min", DGC_PARAM_DOUBLE, &settings.kf_settings.confidence_publish_min, 1, NULL },
//
//      {"transform", "velodyne", DGC_PARAM_TRANSFORM,   &velodyne_offset, 1, NULL},
//      {"velodyne", "cal_file", DGC_PARAM_FILENAME, &cal_filename, 0, NULL},
//      {"imagery", "root", DGC_PARAM_FILENAME, &imagery_root, 0, NULL},
//  };
//  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));

  Param params[] = {
      {"transform", "velodyne", DGC_PARAM_TRANSFORM,   &velodyne_offset, 1, NULL},
      {"velodyne", "cal_file", DGC_PARAM_FILENAME, &cal_filename, 0, NULL},
      {"imagery", "root", DGC_PARAM_FILENAME, &imagery_root, 0, NULL},

      {"perception", "velodyne_threshold_factor", DGC_PARAM_DOUBLE,   &velodyne_threshold_factor, 1, NULL },
      {"perception", "velodyne_max_range", DGC_PARAM_DOUBLE,   &velodyne_max_range, 1, NULL },
      {"perception", "velodyne_min_beam_diff", DGC_PARAM_DOUBLE,   &velodyne_min_beam_diff, 0, NULL },
  };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
}

int main(int argc, char **argv)
{
  char index_filename[300];
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

  /* open velodyne file */
  velodyne_file = dgc_velodyne_open_file(vlf_filename);
  if(velodyne_file == NULL)
    dgc_die("Error: Could not open velodyne file %s for reading.\n", 
        vlf_filename);

  /* load the velodyne index */
  velodyne_index.load(index_filename);

  /* load velodyne calibration & transform */
  dgc_velodyne_get_config(&velodyne_config);
  if(dgc_velodyne_read_calibration(cal_filename, velodyne_config) != 0) {
    fprintf(stderr, "# ERROR: could not read calibration file!\n");
    exit(0);
  }
  dgc_velodyne_integrate_offset(velodyne_offset, velodyne_config);

  spin.load(velodyne_file, velodyne_config, &velodyne_index, current_spin_num,
      &applanix_lat, &applanix_lon, &applanix_alt);

  load_labels();

  /* allocate rolling grid */
//  default_map_cell =
//      (dgc_perception_map_cell_p)calloc(1, sizeof(dgc_perception_map_cell_t));
//  dgc_test_alloc(default_map_cell);
//
//  default_map_cell->max       = -FLT_MAX;
//  default_map_cell->min       = FLT_MAX;
//  default_map_cell->hits      = 0;
//  default_map_cell->seen      = 0;
//  default_map_cell->last_min  = 0;
//  default_map_cell->last_max  = 0;
//  default_map_cell->last_use  = 0;
//  default_map_cell->last_mod  = 0;
//  default_map_cell->dynamic   = 0;
//  default_map_cell->region    = 0;
//  default_map_cell->obstacle  = PERCEPTION_MAP_OBSTACLE_FREE;
//  default_map_cell->street    = 1;
//
//  default_terrain_cell =
//      (dgc_perception_map_cell_p)calloc(1, sizeof(dgc_perception_map_cell_t));
//  dgc_test_alloc(default_terrain_cell);
//
//  *default_terrain_cell = *default_map_cell;
//
//  grid_stat.mapsize.x   = (int) (settings.map_size_x/settings.map_resolution);
//  grid_stat.mapsize.y   = (int) (settings.map_size_y/settings.map_resolution);
//  grid_stat.resolution  = settings.map_resolution;
//  grid_stat.center.x    = 0;
//  grid_stat.center.y    = 0;
//
//  fprintf( stderr, "# INFO: initialize grid map (%.1fm x %.1fm - %.2fm resolution)\n",
//      settings.map_size_x, settings.map_size_y, settings.map_resolution );
//
//  grid =
//      dgc_grid_initialize( grid_stat.resolution,
//          grid_stat.mapsize.x,
//          grid_stat.mapsize.y,
//          sizeof(dgc_perception_map_cell_t),
//          default_map_cell );
//

//
//  perception_init();

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
