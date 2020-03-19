#include "velodyne.h"

#ifdef MULTITHREAD
#include "omp.h"
#endif

#define    MAX_NUM_VELODYNE_SCANS   50000
#define    BINS_PER_REV             720
#define    MAX_BEAMS_IN_BIN         10
#define    MAX_POINTS_PER_SCAN      8000
#define    CM_TO_METER_FACTOR       0.01
#define    VELODYNE_MIN_RANGE       2.0
#define    VELODYNE_MIN_RANGE_S     200
#define    NO_HEIGHT               -100.0
#define    MIN_DISTANCE_IN_CM       100
#define    VELO_BLIND_SPOT_START    17000
#define    VELO_BLIND_SPOT_STOP     19000
#define    MAX_COUNTER_DIFF         15

dgc_transform_t                 velodyne_offset;

float      velodyne_upper_block_max_range_diff  = 200.0;
float      velodyne_lower_block_max_range_diff  = -15.0;

typedef struct {
  int                 num_beams;
  laser_point_p     * beam;
} beam_bin_t;

beam_bin_t           bin[NUM_LASER_BEAMS][BINS_PER_REV];
laser_scan_p         lscan;
laser_scan_p         nscan;

typedef struct {
  int     idx;      /* velodyne index beam */
  int     pb;       /* partner beam for the comparison */
  float   v_angle;  /* vertical angle of the laser beam */
  float   h_angle;  /* horizontal angle of the laser beam */
  int     h_offset; /* horizontal offset of the beam in velodyne ticks */
  float   fac;      /* approximation factor of function with pb */
} velodyne_ring_settings_t;

dgc::VelodyneRings* rings;

#define OCCUPIED    1
#define FREE        0

#define NUM_SAMPLES    1000
#define EPSILON        0.000000001
#define MAX_RANGE      70.0

typedef __gnu_cxx::hash_multimap<uintptr_t, uintptr_t> map_type;

#ifdef USE_GRID_SEGMENTER
map_type cell_to_points;
#endif

void display_time(char* label, double time)
{
  int ms = (int)((dgc_get_time() - time) * 1000);
  if (strlen(label) < 9)
    printf("#TIME: %s\t\t%02d", label, ms);
  else
    printf("#TIME: %s\t%02d", label, ms);

  if (ms > 20)
    printf(" * ");

  printf("\n");
}

void 
velodyne_init( dgc_velodyne_data_p v )
{
  v->num_scans = 0;
  v->scans = 
      (dgc_velodyne_scan_p) malloc( MAX_NUM_VELODYNE_SCANS *
          sizeof(dgc_velodyne_scan_t));
  v->allocated_scans = MAX_NUM_VELODYNE_SCANS;

  dgc_velodyne_get_config(&v->config);
}

void
set_cell_min( dgc_perception_map_cell_p cell,  float z, unsigned short counter )
{
  assert(cell != NULL);
  if (counter_diff(cell->last_min,counter)>MAX_COUNTER_DIFF ||
      cell->min > z) {
    cell->min = z;
    cell->last_min = counter;
  }
}

void
set_cell_max( dgc_perception_map_cell_p cell,  float z, unsigned short counter )
{
  assert(cell != NULL);

  if (counter_diff(cell->last_max,counter)>5) {
    cell->max = z;
    cell->last_max = counter;
  } else if (z > cell->max && z-cell->min<settings.overpass_height) {
    cell->max = z;
    cell->last_max = counter;
  }
}

void
sync_with_terrain( dgc_perception_map_cell_p t, dgc_perception_map_cell_p c )
{

  if(t != NULL && c != NULL) {
    if (t->min<c->min) {
      c->min=t->min;
    }
    if (0 && t->max>c->max) {
      c->max=t->max;
    }
  }
}

void
velodyne_smooth_data( dgc_velodyne_data_p v, int s_upper, int s_lower )
{
  int    l, i;
  float  r0, r1, r2;

  /* upper block */
  if (s_upper) {
    for(l=0; l<32; l++) {
      r0 = lscan[l].laser_point[0].point->range;
      for(i = 1; i < lscan[l].num_points-1; i++) {
        if (!v->config->laser_enabled[l])
          lscan[l].laser_point[i].valid = FALSE;
        r1 = lscan[l].laser_point[i].point->range;
        r2 = lscan[l].laser_point[i+1].point->range;
        if ( r1>MIN_DISTANCE_IN_CM &&
            fabs(r1-r0)<velodyne_upper_block_max_range_diff &&
            fabs(r2-r1)<velodyne_upper_block_max_range_diff ) {
          lscan[l].laser_point[i].point->range = (int)floor((r0+r1+r2)/3.0);
        } else {
          lscan[l].laser_point[i].valid = FALSE;
        }
        r0 = r1;
      }
    }
  }
  /* lower block */
  if (s_lower) {
    for(l=32; l<64; l++) {
      r0 = lscan[l].laser_point[0].point->range * CM_TO_METER_FACTOR;
      for(i = 1; i < lscan[l].num_points-1; i++) {
        if (!v->config->laser_enabled[l])
          lscan[l].laser_point[i].valid = FALSE;
        r1 = lscan[l].laser_point[i].point->range;
        r2 = lscan[l].laser_point[i+1].point->range ;
        if ( r1>MIN_DISTANCE_IN_CM &&
            fabs(r1-r0)<velodyne_lower_block_max_range_diff &&
            fabs(r2-r1)<velodyne_lower_block_max_range_diff ) {
          lscan[l].laser_point[i].point->range = (int)floor(0.2*r0 + 0.6*r1 + 0.2*r2);
        } else {
          lscan[l].laser_point[i].valid = FALSE;
        }
        r0 = r1;
      }
    }
  }

}


/*
 * Used to filter obviously bad returns.
 * Assume the ground slopes down at no more than 11 degrees
 * and that all returns below ground are invalid
 *
 * Values are recalculated directly from config file on each load.
 * These values are from a particular config file (ID89.cal)
 */
unsigned short max_valid_range[] =
{
  0,    0,    0,    0,    0,    0,    0,    0,
  0,    0,    0,    0,    0,    0,    0,    0,
  0,    0,    0,    0,    0,    0,    0,    0,
  0,    0,    0,    0,    0,    0,    0,    0,
  1025, 1070, 0,    0,    1117, 1171, 868,  904,
  1245, 1336, 940,  996,  2225, 2400, 1431, 1473,
  2650, 3002, 1582, 1676, 3542, 4293, 1841, 2007,
  0,    0,    5453, 6522, 0,    0,    8513, 13529
};


void 
initialize(dgc_velodyne_data_p v)
{
  int                          i, l; //, size;

#ifdef MULTITHREAD
  omp_set_dynamic(0);
  omp_set_num_threads(settings.num_threads);
#endif

  for (l=0; l<NUM_LASER_BEAMS; l++) {
    for (i=0; i<BINS_PER_REV; i++) {
      bin[l][i].num_beams = 0;
      bin[l][i].beam      =
          (laser_point_p *) malloc( MAX_BEAMS_IN_BIN * sizeof(laser_point_p) );
    }
  }

  lscan = (laser_scan_p) malloc(NUM_LASER_BEAMS * sizeof(laser_scan_t));
  nscan = (laser_scan_p) malloc(NUM_LASER_BEAMS * sizeof(laser_scan_t));
  for (l=0; l<NUM_LASER_BEAMS; l++) {
    lscan[l].num_points = 0;
    lscan[l].laser_point =
        (laser_point_p) malloc( MAX_POINTS_PER_SCAN * sizeof(laser_point_t) );

    nscan[l].num_points = 0;
    nscan[l].laser_point =
        (laser_point_p) malloc( MAX_POINTS_PER_SCAN * sizeof(laser_point_t) );
  }

  rings = new dgc::VelodyneRings(v->config, settings.velodyne_min_beam_diff);

  double theta = atan2(0.2, 1);
  double L = M_PI_2 + theta;
  double velodyne_height = v->config->offset[2][3] + DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT;
  for (l=0; l<NUM_LASER_BEAMS; l++) {
    double R = v->config->vert_angle[l];
    if (R > -theta) {
      max_valid_range[l] = 0;
      continue;
    }
    R = M_PI_2 + R;

    double max_ground_range = 100.0 * velodyne_height * sin(L) / sin(M_PI - R - L);
    if (max_ground_range > std::numeric_limits<short>::max())
      max_valid_range[l] = 0;
    else
      max_valid_range[l] = (unsigned short) (max_ground_range);
  }
}

void process_velodyne(dgc_velodyne_data_p v)
{
  int l, i, j, e, b, n;
  // *******************************************************************
  // *    Set beams to individual laser units
  // *******************************************************************
  if (!v->preprocessed) {
    laser_scan_p tmp = lscan;
    lscan = nscan;
    nscan = tmp;

    if ((velodyne_ts - last_velodyne_ts) > 0.19) {
      fprintf(stderr, "#WARN: dropping scans can cause blind spots! Delay is %f s\n", (velodyne_ts - last_velodyne_ts));
      for (l=0; l<NUM_LASER_BEAMS; l++) {
        lscan[l].num_points = 0;
      }
    }

    for (l=0; l<NUM_LASER_BEAMS; l++) {
      nscan[l].num_points = 0;
    }

    /*  LASER 00-31: upper block 
        LASER 32-64: lower block  */
    double time = dgc_get_time();
//    int count_zero = 0;
//    int count_max = 0;
//    int less_zero = 0;
    for( i = 0; i < v->num_scans; i++) {
      int encoder = (v->scans[i].encoder + VELO_SPIN_START) % VELO_NUM_TICKS; // unwrap back to 0 ... 36000 range
      encoder -= rings->minHorizontalOffset();
//      printf("v->scans[i].encoder: %d\n", v->scans[i].encoder);

      #pragma omp parallel for private(l, e, n)
      for(j = 0; j < 32; j++) {
        unsigned short range = v->scans[i].p[j].range;
        if (range == 0) {
//          count_zero++;
          // TODO: we may not want to throw these all away
          continue;
        }

        l = j + v->scans[i].block * 32;

        if ((max_valid_range[l]) && (range > max_valid_range[l])) {
//          count_max++;
          continue;
        }

        e = encoder + rings->horizontalOffset(l);
        assert (e >= 0);
        if (e >= VELO_NUM_TICKS) {
          e -= VELO_NUM_TICKS;
          n = nscan[l].num_points;
          if (n<MAX_POINTS_PER_SCAN) {
            nscan[l].laser_point[n].scan       = &(v->scans[i]);
            nscan[l].laser_point[n].point      = &(v->scans[i].p[j]);
  //          nscan[l].laser_point[n].cell       = NULL;
            nscan[l].laser_point[n].z_cell     = NULL;
            nscan[l].laser_point[n].encoder    = e;
            nscan[l].laser_point[n].obstacle   = FALSE;
            if (!v->config->laser_enabled[l])
              nscan[l].laser_point[n].valid    = FALSE;
            else
              nscan[l].laser_point[n].valid    = TRUE;
            nscan[l].num_points++;
          }
          nscan[l].robot = &(v->scans[i].robot);
        } else {
          n = lscan[l].num_points;
          if (n<MAX_POINTS_PER_SCAN) {
            lscan[l].laser_point[n].scan       = &(v->scans[i]);
            lscan[l].laser_point[n].point      = &(v->scans[i].p[j]);
  //          lscan[l].laser_point[n].cell       = NULL;
            lscan[l].laser_point[n].z_cell     = NULL;
            lscan[l].laser_point[n].encoder    = e;
            lscan[l].laser_point[n].obstacle   = FALSE;
            if (!v->config->laser_enabled[l])
              lscan[l].laser_point[n].valid    = FALSE;
            else
              lscan[l].laser_point[n].valid    = TRUE;

            lscan[l].num_points++;
          }
          lscan[l].robot = &(v->scans[i].robot);
        }
      }
    }
    display_time("Projecting", time);
    time = dgc_get_time();
    // *************************************************************
    // * Put beams in bins
    // *************************************************************
    #pragma omp parallel for private(i)
    for (l=0; l<NUM_LASER_BEAMS; l++) {
      for (i=0; i<BINS_PER_REV; i++) {
        bin[l][i].num_beams = 0;
      }
    }

    #pragma omp parallel for private(i,e,b,n)
    for(l=0; l<NUM_LASER_BEAMS; l++) {
      for(i=0; i<lscan[l].num_points; i++) {
        if (lscan[l].laser_point[i].valid) {
          e = lscan[l].laser_point[i].encoder;
          b = (int)floor(e / ((float)VELO_NUM_TICKS/(float)BINS_PER_REV));
          assert(bin[l][b].num_beams < MAX_BEAMS_IN_BIN);
          bin[l][b].beam[bin[l][b].num_beams++] = &lscan[l].laser_point[i];
        }
      }
    }
    display_time("Binning", time);
//    printf("zeros: %d\n", count_zero);
//    printf("max filtered: %d\n", count_max);

    v->preprocessed = TRUE;

  } /* end if (!v->preprocessed) */
}

void label_obstacle_points_rings()
{
  int l, b, l0, l1;
//  int c;
//  float threshold, range0, range1;

  double max_range = settings.velodyne_max_range * 100.0;
  double factor = 0.01 * settings.velodyne_threshold_factor;

//  c = 0;
  #pragma omp parallel for private(l, l0, l1, b)
  for (l=0; l<NUM_LASER_BEAMS-1; l++) {
    if (l != rings->partnerBeam(l)) {
      l1 = rings->beamToIndex(l); // ring[l].idx;            /* shorter beam */
      l0 = rings->partnerIndex(l); // ring[ring[l].pb].idx;   /* longer beam */
      for( b = 0; b < BINS_PER_REV; b++) {
        for(int k=0; k<bin[l0][b].num_beams; k++) {
          float range0 = bin[l0][b].beam[k]->point->range;
          if (bin[l0][b].beam[k]->valid && (range0 < max_range * 100.0)) {
            for(int m=0; m<bin[l1][b].num_beams; m++) {
              if (bin[l1][b].beam[m]->valid) {
                float range1 = bin[l1][b].beam[m]->point->range;
                float threshold = rings->factor(l) * range1 * range1 * factor;
                if ( fabs(range1-range0) < threshold ) {
                  bin[l0][b].beam[k]->obstacle = TRUE;
//                  bin[l1][b].beam[m]->obstacle = TRUE;
//                  c++;
//                  break;
                }
              }
            }
          }
        }
      }
    }
  }
//  printf("%d points labeled as obstacles\n", c);
}

void label_obstacle_points_threshold(double ground_threshold)
{
  short th = short(ground_threshold * 100);
//
  int count = 0;
  int total = 0;
  for (int l=0; l<NUM_LASER_BEAMS; l++) {
    total += lscan[l].num_points;
    for (int k=0; k < lscan[l].num_points; k++) {
      lscan[l].laser_point[k].obstacle = (lscan[l].laser_point[k].point->z > th);
      count += (lscan[l].laser_point[k].point->z > th);
    }
  }
//  printf("%d / %d points are obstacles\n", count, total);
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

unsigned short min_ground_range_adjusted[] =
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

void label_obstacle_points_range() {
//  static bool init = true;
//  if (init) {
//    for (int i=0; i < NUM_LASER_BEAMS; i++)
//      min_ground_range_adjusted[i] = (short)( min_ground_range[i] * 0.75);
//    init = false;
//  }
//
//  int l;
//
//  #pragma omp parallel for
//  for (l=0; l<NUM_LASER_BEAMS; l++) {
//    for (int i = 0; i < lscan[l].num_points; i++) {
//      short r = lscan[l].laser_point[i].point->range;
//      int b = rings->indexToBeam(l); //ridx[l];
//      if ((r < min_ground_range_adjusted[b]) || (r > max_ground_range))
//        lscan[l].laser_point[i].obstacle = 1;
//    }
//  }
}

//TODO: cut from 8ms to 5ms
void label_obstacle_points_terrain() {

// EXPAND_Z_GRID adds about 3ms to processing time, and
// it does a single step of expansion on z_grid
//#define EXPAND_Z_GRID

#ifdef EXPAND_Z_GRID
  __gnu_cxx::hash_set<int> open;
#endif

  static bool init = true;
  static short obstacle_threshold = 0;
  if (init) {
    obstacle_threshold = (short)(settings.z_obstacle_height / 0.01);
    init = false;
  }

  static short max_z = std::numeric_limits<short>::max();
  int l;
  #pragma omp parallel for
  for (l=0; l<NUM_LASER_BEAMS; l++) {
    for (int i = 0; i < lscan[l].num_points; i++) {
      short* grid_z = (short*)dgc_grid_get_xy(z_grid, lscan[l].laser_point[i].point->x, lscan[l].laser_point[i].point->y);
      if (grid_z) {
        *grid_z = max_z;
        lscan[l].laser_point[i].z_cell = grid_z;
      }
    }
  }

  #pragma omp parallel for
  for (l=0; l<NUM_LASER_BEAMS; l++) {
    for (int i = 0; i < lscan[l].num_points; i++) {
      short* grid_z = lscan[l].laser_point[i].z_cell;
      if (grid_z) {
        *grid_z = std::min(*grid_z, lscan[l].laser_point[i].point->z);
#ifdef EXPAND_Z_GRID
        open.insert((int)grid_z);
#endif
      }
    }
  }

//  printf("open size: %d\n", open.size());

#ifdef EXPAND_Z_GRID
  __gnu_cxx::hash_set<int>::iterator end = open.end();
  for (__gnu_cxx::hash_set<int>::iterator it = open.begin(); it != end; it++) {
    short* cell = (short*) *it;
    //    printf("processing %d %d\n", (int)cell, *cell);
    int r,c;
    dgc_grid_cell_to_rc_local(z_grid, cell, &r, &c);

    // look at four neighborhood
    short* neighbor = (short*)dgc_grid_get_rc_local(z_grid, r+1, c);
//    if ((neighbor) && (*cell > (*neighbor + 20)))
    if ((open.find((int)neighbor)!=end) && (*cell > (*neighbor + 20)))
      *cell = *neighbor + 20;

    neighbor = (short*)dgc_grid_get_rc_local(z_grid, r-1, c);
    if ((open.find((int)neighbor)!=end) && (*cell > (*neighbor + 20)))
      *cell = *neighbor + 20;

    neighbor = (short*)dgc_grid_get_rc_local(z_grid, r, c-1);
    if ((open.find((int)neighbor)!=end) && (*cell > (*neighbor + 20)))
      *cell = *neighbor + 20;

    neighbor = (short*)dgc_grid_get_rc_local(z_grid, r, c+1);
    if ((open.find((int)neighbor)!=end) && (*cell > (*neighbor + 20)))
      *cell = *neighbor + 20;
  }
#endif

  #pragma omp parallel for
  for (int l=0; l<NUM_LASER_BEAMS; l++) {
    for (int i = 0; i < lscan[l].num_points; i++) {
      short z = lscan[l].laser_point[i].point->z;
      short* grid_z = lscan[l].laser_point[i].z_cell;
      if ((grid_z) && ( (z - *grid_z) > obstacle_threshold))
         lscan[l].laser_point[i].obstacle = 1;
    }
  }
}

//void label_obstacle_points_terrain_range()
//{
//  static bool init = true;
//  static short obstacle_threshold = 0;
//  if (init) {
//    obstacle_threshold = (short)(settings.terrain_obstacle_height / 0.01);
//
//    for (int i=0; i < NUM_LASER_BEAMS; i++)
//      min_ground_range_adjusted[i] = (short)( min_ground_range[i] * 0.8);
//
//    init = false;
//  }
//
//  dgc_grid_clear(z_grid);
//  for (int l=0; l<NUM_LASER_BEAMS; l++) {
//    for (int i = 0; i < lscan[l].num_points; i++) {
//      if (!lscan[l].laser_point[i].valid)
//        continue;
//      double x = lscan[l].laser_point[i].point->x * CM_TO_METER_FACTOR;
//      double y = lscan[l].laser_point[i].point->y * CM_TO_METER_FACTOR;
//      short z = lscan[l].laser_point[i].point->z;
//      short* grid_z;
//      grid_z = (short*)dgc_grid_get_xy(z_grid, x, y);
//      if (grid_z)
//        *grid_z = std::min(*grid_z, z);
//      lscan[l].laser_point[i].z_cell = grid_z;
//    }
//  }
//
//  for (int l=0; l<NUM_LASER_BEAMS; l++) {
//    for (int i = 0; i < lscan[l].num_points; i++) {
//      short z = lscan[l].laser_point[i].point->z;
//      short* grid_z = lscan[l].laser_point[i].z_cell;
//      if ((grid_z) && ( (z - *grid_z) > obstacle_threshold))
//         lscan[l].laser_point[i].obstacle = 1;
//      else {
//        short r = lscan[l].laser_point[i].point->range;
//        int b = ridx[l];
//        if ((r < min_ground_range_adjusted[b]) || (r > max_ground_range))
//          lscan[l].laser_point[i].obstacle = 1;
//      }
//    }
//  }
//}

// TODO: cut from 30ms to 10ms
void label_obstacle_points()
{
  double time = dgc_get_time();
  label_obstacle_points_rings();
  display_time("Rings", time);
  time = dgc_get_time();
  label_obstacle_points_range();
  display_time("Range", time);
  time = dgc_get_time();
  label_obstacle_points_terrain();
  display_time("Z", time);
}

void report_num_threads(int level)
{
//  #pragma omp single
//  {
//    printf("Level %d: number of threads in the team - %d\n", level, omp_get_num_threads());
//  }
}

/**
 * TODO: speed this up (by at least a factor of 2)
 * IDEAS:
 *   1) X get rid of terrain_grid (use z-grid if we really need to)
 *   2) X get rid of map_s (called by include_cell)
 *   3) get rid of min range filtering
 *   4) X get rid of cell_to_points (only OK if we go to Laser Segmenter)
 *   5) X speed up grid_get_xy
 *   6) move local to global transform ( * CM_TO_METER_FUN + scan->robot.x)  outside (we're doing the same work in Obstacle.addPoint())
 *   7) OMP Parallel
 *   8) Get rid of overpass checking
 */

void label_obstacle_cells(unsigned short counter, dgc_velodyne_data_p v)
{
  int l, b, k;
  float x, y, z, h;
  dgc_perception_map_cell_p cell;

  double time = dgc_get_time();
#ifdef USE_GRID_SEGMENTER
  cell_to_points.clear();
#endif
//  int c = 0;
  #pragma omp parallel for private(b,k,x,y,z,cell) shared(counter)
  for (l=0; l<NUM_LASER_BEAMS-1; l++) {
    for(b = 0; b < BINS_PER_REV; b++) {
      for(k=0; k<bin[l][b].num_beams; k++) {

        if (bin[l][b].beam[k]->valid) {
          x = bin[l][b].beam[k]->point->x * CM_TO_METER_FACTOR + bin[l][b].beam[k]->scan->robot.x;
          y = bin[l][b].beam[k]->point->y * CM_TO_METER_FACTOR + bin[l][b].beam[k]->scan->robot.y;
          cell = (dgc_perception_map_cell_p)grid_get_xy(grid, x, y );
          if(cell) {

#ifdef USE_GRID_SEGMENTER
          #pragma omp critical
          {
            cell_to_points.insert(map_type::value_type((uintptr_t)cell, (uintptr_t)bin[l][b].beam[k]));

	    ++cell->num_timestamps;
	    cell->timestamp_sum += bin[l][b].beam[k]->scan->timestamp;
          }
#endif

//            c++;
            z = bin[l][b].beam[k]->point->z * CM_TO_METER_FACTOR + bin[l][b].beam[k]->scan->robot.z;
            set_cell_min( cell, z, counter );
            set_cell_max( cell, z, counter );

            cell->last_observed = counter;
            if (cell->last_obstacle != counter) {
              h = z - cell->min;
              if (h<settings.overpass_height) {
                if (bin[l][b].beam[k]->point->range > VELODYNE_MIN_RANGE_S) {
//                  printf("%d, %d\n", bin[l][b].beam[k]->point->x, bin[l][b].beam[k]->point->y );
                  if (bin[l][b].beam[k]->obstacle) {
                    #pragma omp critical
                    {
                      obstacles_s->cell[obstacles_s->num++] = cell;
                    }

                    cell->last_obstacle = counter;
                    cell->hits += 2; //= settings.map_cell_increase;
                  } else {
                    cell->seen ++;
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  assert(obstacles_s->num < MAX_NUM_POINTS);
  display_time("Cells", time);
//  printf("labeled %d cells as obstacles\n", obstacles_s->num);
}

// Add obstacles near car that the Velodyne can't see
void label_obstacle_cells_near(int counter, dgc_velodyne_data_p v) {
  static const int near_obstacle_buffer = 10;
  static const float near_obstacle_radius = 4.2;

  double time = dgc_get_time();

  int origin_r, origin_c;
  dgc_transform_t t;
  double velodyne_x = 0.0, velodyne_y = 0.0, velodyne_z = 0.0;

  dgc_transform_rpy(t, v->config->offset, v->scans[0].robot.roll, v->scans[0].robot.pitch, v->scans[0].robot.yaw);
  dgc_transform_translate(t, v->scans[0].robot.x, v->scans[0].robot.y, v->scans[0].robot.z);
  dgc_transform_point(&velodyne_x, &velodyne_y, &velodyne_z, t);


  float velodyne_height = v->config->offset[2][3] + DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT;
  // TODO: figure out why we need to add 40 cm here to get a realistic ground_z
  float ground_z = applanix_current_pose()->smooth_z - DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT + v->config->offset[2][3];


  dgc_perception_map_cell_p origin_cell = (dgc_perception_map_cell_p)grid_get_xy(grid, velodyne_x, velodyne_y );
  dgc_grid_cell_to_rc_local(grid, origin_cell, &origin_r, &origin_c);

  int invisible_radius = ceil(near_obstacle_radius / grid->resolution);
  int invisible_radius2 = invisible_radius * invisible_radius;
  for (int r = -invisible_radius; r <= invisible_radius; r++) {
    int r2 = r*r;
    for (int c = -invisible_radius; c <= invisible_radius; c++) {
      int c2 = c*c;
      int d2 = (r2 + c2);
      if (d2 > invisible_radius2)
        continue;

      dgc_perception_map_cell_p cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local(grid, origin_r + r, origin_c + c);
      if ( cell &&
          (cell->last_observed != counter) &&
          (cell->last_obstacle > 0) &&
          ((cell->last_observed - cell->last_obstacle) < near_obstacle_buffer) &&
          ((cell->last_obstacle - cell->last_dynamic) > near_obstacle_buffer) &&
          (cell->hits > 2)) {
        // remove obstacles that we should be able to see because of their height,
        // this solves the problem of high obstacles passing through/near the occluded area and leaving behind static obstacles
        double distance = sqrt(d2) * grid->resolution;
        double max_z = ( (near_obstacle_radius-distance) / near_obstacle_radius) * velodyne_height + ground_z;

        // following 2 lines are for visualization purposes only:
//        cell->max = max_z;
//        cell->min = ground_z;

        if (cell->max > max_z)
          continue;

        obstacles_s->cell[obstacles_s->num] = cell;
        if (obstacles_s->num<MAX_NUM_POINTS) {
          obstacles_s->num++;
        }
      }
    }
  }

  display_time("Cells Near", time);
}

void points_in_cell(dgc_perception_map_cell_p cell, std::vector<point3d_t>& points) {
#ifdef USE_GRID_SEGMENTER
  points.reserve(cell_to_points.count((uintptr_t)cell));
  __gnu_cxx::pair<map_type::iterator, map_type::iterator> p = cell_to_points.equal_range((uintptr_t)cell);
  point3d_t point;
  for (map_type::iterator it = p.first; it != p.second; it++) {
    laser_point_p pt = (laser_point_p)(it->second);
    point.x = pt->point->x * CM_TO_METER_FACTOR + pt->scan->robot.x;
    point.y = pt->point->y * CM_TO_METER_FACTOR + pt->scan->robot.y;
    point.z = pt->point->z * CM_TO_METER_FACTOR + pt->scan->robot.z;
    point.intensity = pt->point->intensity;
    points.push_back(point);
  }
#endif
}


//void label_obstacle_points_from_cells()
//{
//  // create a hash look up from obstacle cells
//  __gnu_cxx::hash_set<int> obstacle_set;
//
//  for (int i=0; i < obstacles_s->num; i++) {
//    obstacle_set.insert((int)obstacles_s->cell[i]);
//  }
//
//  int c = 0;
//  __gnu_cxx::hash_set<int>::iterator end =  obstacle_set.end();
//  for (int l=0; l<NUM_LASER_BEAMS; l++) {
//    for (int k=0; k < lscan[l].num_points; k++) {
//      lscan[l].laser_point[k].obstacle = (obstacle_set.find((int)lscan[l].laser_point[k].cell) != end);
//    }
//  }
//
//  printf("%d points labeled as obstacles\n", c);
//}

void
integrate_velodyne( dgc_velodyne_data_p v, unsigned short counter )
{
  static int                   firsttime = 1;

  if (v->num_scans==0)
    return;

  if (firsttime) {
    initialize(v);
    firsttime = 0;
  }

  process_velodyne(v);
  label_obstacle_points();
  label_obstacle_cells(counter, v);
  label_obstacle_cells_near(counter, v);

//  segment_velodyne();
//  draw_segments();
//  draw_velodyne();
}

void
test_clusters(int num_scans, dgc_velodyne_scan_p scans, dgc_velodyne_config_p config, double ground_threshold, std::vector<laser_cluster_p>& clusters, double x, double y)
{
  dgc_velodyne_data_t v;

  v.config = config;
  v.num_scans = num_scans;
  v.allocated_scans = num_scans;
  v.scans = scans;
  v.preprocessed = 0;

  static int                   firsttime = 1;

  if (v.num_scans==0)
    return;

  if (firsttime) {
    initialize(&v);
    firsttime = 0;
  }

  process_velodyne(&v);
  label_obstacle_points_threshold(ground_threshold);
//  test_segment_velodyne(x, y);
//  collect_obstacles(clusters);
}


dgc_pose_t computeAverageRobotPoseForCell(dgc_perception_map_cell_p cell)
{
  #ifndef USE_GRID_SEGMENTER
  bool this_function_requires_grid_segmenter = false;
  assert(this_function_requires_grid_segmenter);
  #endif

  dgc_pose_t robot;
  robot.x = 0;
  robot.y = 0;
  robot.z = 0;
  robot.roll = 0;
  robot.pitch = 0;
  robot.yaw = 0;
  
  double count = 0;
  __gnu_cxx::pair<map_type::iterator, map_type::iterator> p = cell_to_points.equal_range((uintptr_t)cell);
  for (map_type::iterator it = p.first; it != p.second; it++) {
    laser_point_p pt = (laser_point_p)(it->second);
    robot.x += pt->scan->robot.x;
    robot.y += pt->scan->robot.y;
    robot.z += pt->scan->robot.z;
    robot.roll += pt->scan->robot.roll;
    robot.pitch += pt->scan->robot.pitch;
    robot.yaw += pt->scan->robot.yaw;

    ++count;
  }

  robot.x /= count;
  robot.y /= count;
  robot.z /= count;
  robot.roll /= count;
  robot.pitch /= count;
  robot.yaw /= count;

  return robot;
}


dgc_pose_t computeAverageRobotPoseForCells(const std::vector<dgc_perception_map_cell_p>& cells)
{
  #ifndef USE_GRID_SEGMENTER
  bool this_function_requires_grid_segmenter = false;
  assert(this_function_requires_grid_segmenter);
  #endif

  dgc_pose_t robot;
  robot.x = 0;
  robot.y = 0;
  robot.z = 0;
  robot.roll = 0;
  robot.pitch = 0;
  robot.yaw = 0;
  
  double count = 0;
  for(size_t i = 0; i < cells.size(); ++i) {
    __gnu_cxx::pair<map_type::iterator, map_type::iterator> p = cell_to_points.equal_range((uintptr_t)cells[i]);
    for (map_type::iterator it = p.first; it != p.second; it++) {
      laser_point_p pt = (laser_point_p)(it->second);
      robot.x += pt->scan->robot.x;
      robot.y += pt->scan->robot.y;
      robot.z += pt->scan->robot.z;
      robot.roll += pt->scan->robot.roll;
      robot.pitch += pt->scan->robot.pitch;
      robot.yaw += pt->scan->robot.yaw;

      ++count;
    }
  }

  assert(count != 0);
  robot.x /= count;
  robot.y /= count;
  robot.z /= count;
  robot.roll /= count;
  robot.pitch /= count;
  robot.yaw /= count;

  return robot;
}
