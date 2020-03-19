#include <roadrunner.h>
#include <param_interface.h>
#include <grid.h>
#include <lltransform.h>
#include <ipc_std_interface.h>
#include "velo_support.h"
#include "terrainmap.h"

#define MAX_RANGE 50.0

using namespace dgc;

IpcInterface   *ipc = NULL;



/* parameters */

char *cal_filename = NULL;
char *int_filename = NULL;
int  calibrate_intensities = 1;
dgc_transform_t velodyne_offset;

int first = 1;
double last_x = 0, last_y = 0;

typedef struct {
  terrain_tile *tile;
} grid_cell_t;

dgc_grid_p tile_grid = NULL;

FILE *rndf = NULL;

void rndf_create(char *name) {
	char fname[255];
	sprintf(fname, "%s.rndf", name);
	rndf = fopen(fname, "w");
	fprintf(rndf, "RNDF_name %s\n", name);
	fprintf(rndf, "num_segments\t1\n");
	fprintf(rndf, "num_zones\t0\n");
	fprintf(rndf, "format_version\t1.0\n");
	fprintf(rndf, "creation_date\t06/25/2009\n");
	fprintf(rndf, "segment\t1\n");
	fprintf(rndf, "num_lanes\t1\n");
	fprintf(rndf, "lane\t1.1\n");
	fprintf(rndf, "num_waypoints\t2000\n");
	fprintf(rndf, "lane_width\t12\n");
}

void rndf_add_waypoint(ApplanixPose *pose) {
	static int w_num = 1;
	static float last_yaw = 0.0;
	static double last_x = 0.0, last_y = 0.0;
	double utm_x = 0, utm_y = 0;
	char utmzone[5];
  	vlr::latLongToUtm(pose->latitude, pose->longitude, &utm_x, &utm_y, utmzone);
	float dist = hypot(utm_x - last_x, utm_y - last_y);
	if(dist > 2.0 && (fabs(pose->yaw - last_yaw) > M_PI / 60 || dist > 30.0)) {
		printf("1.1.%d\t%.7lf\t%.7lf\n", w_num, pose->latitude, pose->longitude);
		fprintf(rndf, "1.1.%d\t%.7lf\t%.7lf\n", w_num, pose->latitude, pose->longitude);
		last_x = utm_x;
		last_y = utm_y;
		last_yaw = pose->yaw;
		w_num++;
	} else {
		printf("Skipping waypoint. Distance: %f    Yaw: %f\n", dist, pose->yaw);
	}
}

void rndf_close() {
	fprintf(rndf, "end_lane\n");
	fprintf(rndf, "end_segment\n");
	fclose(rndf);
}

void grid_clear_handler(void *cell) {
  terrain_tile *tile = ((grid_cell_t *)cell)->tile;
  if(tile != NULL) {
    fprintf(stderr, "Saving %s\n", tile->filename);
    tile->save(tile->filename);
    delete ((grid_cell_t *)cell)->tile;
  }
}

int spin_counter = 0;
int move_counter = 0;

void 
my_spin_func( dgc_velodyne_spin *spin, 
	      dgc_velodyne_config_p config, 
	      ApplanixPose *applanix_pose ) 
{
//printf("spin!\n");
  int i, j, r, c, tile_y, tile_x, beam_num, ring_num;
  double p_x, p_y, p_z, utm_x, utm_y;
  grid_cell_t *cell;
  char utmzone[10];
  terrain_tile_cell *grid_cell = NULL;
  static double utm_offset_x = 0, utm_offset_y = 0;
  static double utm_offset_x0 = 0, utm_offset_y0 = 0;
  double u;
  
  spin_counter++;

  /* if(spin_counter > 100)
	return; */

  if(spin->num_scans <= 0) {
    fprintf(stderr, "Warning: spin has zero scans.  Shouldn't happen\n");
    return;
  }

  vlr::latLongToUtm(applanix_pose->latitude, applanix_pose->longitude, 
	      &utm_x, &utm_y, utmzone);

  utm_offset_x = utm_x - applanix_pose->smooth_x;
  utm_offset_y = utm_y - applanix_pose->smooth_y;
  if(first) {
	utm_offset_x0 = utm_offset_x;
	utm_offset_y0 = utm_offset_y;
  }

  if(first || hypot(utm_x - last_x, utm_y - last_y) > 1.0) {
    last_x = utm_x;
    last_y = utm_y;
    dgc_grid_recenter_grid(tile_grid, utm_x, utm_y);
    rndf_add_waypoint(applanix_pose);
  }
  first = 0;

  if(fabs(applanix_pose->speed) < dgc_mph2ms(1.0)) {
	//printf("Skipping, speed is %f\n", applanix_pose->speed);
    return;
  }
//printf("  moving spin!\n");
  move_counter++;

  for(i = 0; i < spin->num_scans; i++) {
    for(j = 0; j < VELO_BEAMS_IN_SCAN; j++) {
      /* skip bad readings */
      if(spin->scans[i].p[j].range < 1.0)
	continue;
      
      if(spin->scans[i].p[j].range * .01 > MAX_RANGE) // skip long readings
	continue;
      beam_num = j + spin->scans[i].block * 32;
      if(spin->scans[i].block == 6) continue;

      ring_num = config->inv_beam_order[beam_num];
      
      /* project beam */
      //p_x = spin->scans[i].p[j].x * 0.01 + spin->scans[i].robot.x;
      //p_y = spin->scans[i].p[j].y * 0.01 + spin->scans[i].robot.y;
      p_x = spin->scans[i].p[j].x * 0.01 + spin->scans[i].robot.x + utm_offset_x;
      p_y = spin->scans[i].p[j].y * 0.01 + spin->scans[i].robot.y + utm_offset_y;
      p_z = spin->scans[i].p[j].z * 0.01 + 0 * spin->scans[i].robot.z;

      /* find the beam's terrain tile */
      r = (int)floor(p_y / tile_grid->resolution);
      c = (int)floor(p_x / tile_grid->resolution);
      cell = (grid_cell_t *)dgc_grid_get_rc_global(tile_grid, r, c);
      if(cell != NULL) {
	/* initialize the tile */
	if(cell->tile == NULL) {
	  cell->tile = new terrain_tile(TERRAIN_TILE_SIZE, TERRAIN_TILE_SIZE);
	  sprintf(cell->tile->filename, "lmap-%s-%d-%d-%06d-%06d.tf.gz", 
		  utmzone, (int)rint(TERRAIN_TILE_RESOLUTION * 100), 
		  TERRAIN_TILE_SIZE, c, r);
	  fprintf(stderr, "Loading %s\n", cell->tile->filename);
	  if(cell->tile->load(cell->tile->filename) < 0) {
	    //	    fprintf(stderr, "  Could not load file.\n");
	    cell->tile->utm_x0 = c * tile_grid->resolution;
	    cell->tile->utm_y0 = r * tile_grid->resolution;
	    cell->tile->resolution = TERRAIN_TILE_RESOLUTION;
	    strcpy(cell->tile->utmzone, utmzone);
	  }
	}
	
	tile_x = 
	  (int)floor((p_x - cell->tile->utm_x0) / TERRAIN_TILE_RESOLUTION);
	tile_y = 
	  (int)floor((p_y - cell->tile->utm_y0) / TERRAIN_TILE_RESOLUTION);
	grid_cell = &(cell->tile->cell[tile_x][tile_y]);

	u = spin->scans[i].p[j].intensity;
	grid_cell->intensity += u;
	grid_cell->i_count++;
	if(i > 3 && i < spin->num_scans - 3) {
		grid_cell->z_count++;
		double dx = spin->scans[i+2].p[j].x - spin->scans[i-2].p[j].x;
		double dy = spin->scans[i+2].p[j].y - spin->scans[i-2].p[j].y;
		double points_theta = atan2(dy, dx);
		double ray_theta = atan2(spin->scans[i].p[j].y, spin->scans[i].p[j].x);
		double curbness = fabs(cos(points_theta - ray_theta));
		curbness *= 1;
		//grid_cell->z += .0005 * fabs(applanix_pose->speed) * pow(curbness, 5) * (pow(.01 * spin->scans[i].p[j].range, 1.0));
		grid_cell->z_response += pow(curbness, 6) * .0005 * fabs(applanix_pose->speed) * spin->scans[i].p[j].range;
		if(p_z < grid_cell->z)
			grid_cell->z = p_z;
		if(p_z > grid_cell->current_highest)
			grid_cell->current_highest = p_z;
	}
	p_z = u;
	if(grid_cell->n == 0) {
		grid_cell->n = 1;
		grid_cell->xa = p_z;
		grid_cell->sxi2 = p_z * p_z;
	} else {
		grid_cell->n++;
		grid_cell->xa = ((grid_cell->n - 1) * grid_cell->xa + p_z) / grid_cell->n;
		grid_cell->sxi2 += p_z * p_z;
	}
	grid_cell->stdev = pow((grid_cell->sxi2 / grid_cell->n - grid_cell->xa * grid_cell->xa), .5);
      }
    }
  }
}

void 
read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = {
    {"transform", "velodyne", DGC_PARAM_TRANSFORM, &velodyne_offset, 0, NULL},
    {"velodyne", "cal_file", DGC_PARAM_FILENAME, &cal_filename, 0, NULL},
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

int 
main(int argc, char **argv) 
{
  grid_cell_t *default_tile = NULL;

  if(argc < 3)
    dgc_die("Error: not enough arguments.\n"
            "Usage: %s log-file velodyne-file\n", argv[0]);

  /* connect to IPC server, get parameters, and disconnect */
  ipc = new IpcStandardInterface;
  if (ipc->Connect("map_velodyne") < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  dgc::ParamInterface  *pint;
  pint = new dgc::ParamInterface(ipc);
  read_parameters(pint, argc, argv);
  ipc->Disconnect();

  default_tile = (grid_cell_t *)calloc(1, sizeof(grid_cell_t));
  dgc_test_alloc(default_tile);

  tile_grid = dgc_grid_initialize(TERRAIN_TILE_RESOLUTION * TERRAIN_TILE_SIZE,
				  TERRAIN_GRID_NUM_TILES, 
				  TERRAIN_GRID_NUM_TILES,
				  sizeof(grid_cell_t), default_tile);
  dgc_grid_set_clear_handler(tile_grid, grid_clear_handler);

  rndf_create("OneLane");
  vlf_projector(argv[2], argv[1], cal_filename, int_filename, velodyne_offset, my_spin_func);
  dgc_grid_clear(tile_grid);
  printf("Map-making complete. Processed %d velodyne spins, %d moving\n", spin_counter, move_counter);
  rndf_close();
  return 0;
}
