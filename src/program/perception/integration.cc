#include "perception.h"
#include "utils.h"
#include "lltransform.h"

/******************************************************************
 * INTEGRATION
 ******************************************************************/
#include "integration_globals.h"

using namespace std;
using namespace std::tr1;
using namespace vlr;
using namespace dgc;

void
use_only_obstacles_with_neighbors( void )
{
  static int            init = TRUE;
  static int            offset[8];
  char                * cptr, * nptr[8];
  static char         * start, * end;
  int                   i, j, ctr = 0;

  if (init) {
    offset[0] = - grid->bytes_per_cell;
    offset[1] =   grid->bytes_per_cell;
    offset[2] = - grid->cols * grid->bytes_per_cell;
    offset[3] =   grid->cols * grid->bytes_per_cell;
    offset[4] =   offset[2] + offset[0];
    offset[5] =   offset[2] + offset[1];
    offset[6] =   offset[3] + offset[0];
    offset[7] =   offset[3] + offset[1];
    start     = (char *)grid->cell;
    end       = (char *)(start + grid->cols * grid->rows * grid->bytes_per_cell);
    init = FALSE;
  }

  for (i=0; i<obstacles_s->num; i++) {
    cptr = (char *)(obstacles_s->cell[i]);
    for (j=0; j<8; j++) {
      nptr[j] = (char *)(cptr + offset[j]);
    }
    for (j=0; j<8; j++) {
      if ( (char *)nptr[j]>=start &&
          (char *)nptr[j]<end   &&
          ((dgc_perception_map_cell_p) nptr[j])->obstacle) {
        obstacles_s->cell[ctr++] = obstacles_s->cell[i];
        break;
      }
    }
  }
  for (i=0; i<obstacles_s->num; i++) {
    obstacles_s->cell[i]->obstacle = FALSE;
  }

  obstacles_s->num = ctr;
}

void 
perception_prep_obstacles( dgc_grid_p grid, 
			      dgc_perception_map_cells_p points, vector< shared_ptr<TrackedObstacle> > obstacles,
			      unsigned short counter )
{
  static double last_time;
  static int                                    first = 1;
  int                                           i, mc, mr, max_num, err;
  short                                         px, py;
  float                                         ll_x, ll_y;

  if(first) {
    strncpy(msg.host, dgc_hostname(), 10);
    msg.point = (PerceptionObstaclePoint *) 
      malloc(MAX_NUM_POINTS * sizeof(PerceptionObstaclePoint));
    msg.dynamic_obstacle = (PerceptionDynamicObstacle *) 
      malloc(MAX_NUM_TRACKER_OBSTACLES * 
	     sizeof(PerceptionDynamicObstacle));
    first = 0;
  }

  msg.timestamp  = points->timestamp;
  msg.counter    = counter;
  mc             = grid->cols/2;
  mr             = grid->rows/2;
  ll_x           = (grid->map_c0)*grid->resolution;
  ll_y           = (grid->map_r0)*grid->resolution;

  msg.num_points = 0;
  printf("Points: %d\n", points->num);
  if (points->num>MAX_NUM_IPC_OBSTACLES) {
    printf("#WARNING: dropping static obstacle points due to IPC constraints!\n");
    max_num = MAX_NUM_IPC_OBSTACLES;
  } else {
    max_num = points->num;
  }
  for (i=0; i<max_num; i++) {
    cell_to_coord( grid,  points->cell[i], &px, &py );
    msg.point[msg.num_points].x     = ll_x + ( (px+0.5) * grid->resolution );
    msg.point[msg.num_points].y     = ll_y + ( (py+0.5) * grid->resolution );

//    msg.point[msg.num_points].type = points->cell[i]->region;
    if (points->cell[i]->last_dynamic == counter) {
      msg.point[msg.num_points].type = DGC_DYNAMIC_OBSTACLE;
    } else {
      msg.point[msg.num_points].type = DGC_STATIC_OBSTACLE;
    }

    msg.point[msg.num_points].z_min = points->cell[i]->min;
    msg.point[msg.num_points].z_max = points->cell[i]->max;
    //if (points->cell[i]->hits > 4)
    msg.num_points++;
//    assert(points->cell[i]->hits > -1);
  }

  int num_obstacles = obstacles.size();
  int num_peds = 0;
  int num_bicycles = 0;
  int num_cars = 0;
  int num_unknown = 0;
  for (i = 0; i < num_obstacles; i++) {
    msg.dynamic_obstacle[i].id = obstacles[i]->id;
    switch (obstacles[i]->type) {
      case OBSTACLE_BICYCLIST:  num_bicycles++; break;
      case OBSTACLE_CAR:        num_cars++;     break;
      case OBSTACLE_PEDESTRIAN: num_peds++;     break;
      default:                  num_unknown++;  break;
    }

    msg.dynamic_obstacle[i].obstacleType = obstacles[i]->type;
    msg.dynamic_obstacle[i].obstacleTypeThisFrame = obstacles[i]->type_this_frame_;
    msg.dynamic_obstacle[i].classifiedThisFrame = obstacles[i]->classified_this_frame_;
    

    msg.dynamic_obstacle[i].x = obstacles[i]->pose.x;
    msg.dynamic_obstacle[i].y = obstacles[i]->pose.y;

    msg.dynamic_obstacle[i].direction = obstacles[i]->pose.yaw;
    msg.dynamic_obstacle[i].velocity = obstacles[i]->getVelocity();
    //msg.dynamic_obstacle[i].velocity = 5;

    msg.dynamic_obstacle[i].length = obstacles[i]->length;
    msg.dynamic_obstacle[i].width = obstacles[i]->width;

    //msg.dynamic_obstacle[i].confidence = max(0.0, min(1.0, obstacles[i]->filter->getPositionUncertainty() * 200));
    msg.dynamic_obstacle[i].confidence = (unsigned char)min(254.9, obstacles[i]->getNumObservations() * 1.0);
    msg.dynamic_obstacle[i].x_var = 0;//obstacles[i]->getXVar();
    msg.dynamic_obstacle[i].y_var = 0;//obstacles[i]->getYVar();
    msg.dynamic_obstacle[i].xy_cov = 0;//obstacles[i]->getXYCov();
  }
  printf("Cars:        %d\n", num_cars);
  printf("Bicycles:    %d\n", num_bicycles);
  printf("Pedestrians: %d\n", num_peds);
  printf("Other:       %d\n", num_unknown);

  /*if (pose->speed < dgc_mph2ms(5.0)) {
    dgc_transform_t t;
    dgc_transform_t* radar;
    for (int r = 0; r < NUM_LRR2_RADARS; r++) {
      switch (r) {
      case 0: radar = &radar_offset[0]; break;
      case 1: radar = &radar_offset[1]; break;
      case 2: radar = &radar_offset[3]; break;
      case 3: radar = &radar_offset[4]; break;
      }

      dgc_transform_copy(t, *radar);
      dgc_transform_rotate_x( t, pose->roll );
      dgc_transform_rotate_y( t, pose->pitch );
      dgc_transform_rotate_z( t, pose->yaw );
      dgc_transform_translate( t, pose->smooth_x, pose->smooth_y, pose->smooth_z );
      double rx = 0; double ry = 0; double rz = 0;
      dgc_transform_point(&rx, &ry, &rz, t);
      for (i = 0; i < radar_lrr2[r].num_targets; i++) {
        RadarTarget* target = &radar_lrr2[r].target[i];
        if (fabs(target->relative_velocity) < dgc_mph2ms(5.0))
          continue;

        msg.dynamic_obstacle[num_obstacles].id = target->id;
        msg.dynamic_obstacle[num_obstacles].obstacleType = 0;

        double x = target->distance + 1.0;
        double y = target->lateral_offset;
        double z = 0.0;
        dgc_transform_point(&x, &y, &z, t);

        msg.dynamic_obstacle[num_obstacles].x = x;
        msg.dynamic_obstacle[num_obstacles].y = y;

        msg.dynamic_obstacle[num_obstacles].velocity = target->relative_velocity;
        msg.dynamic_obstacle[num_obstacles].direction = (target->relative_velocity > 0) ? atan2(y-ry, x-rx) : atan2(y-ry, x-rx) + 2*M_PI;

        msg.dynamic_obstacle[num_obstacles].length = 1.0;
        msg.dynamic_obstacle[num_obstacles].width = 1.0;

        x = 0.0;
        y = target->lateral_offset_var;
        z = 0.0;
        dgc_transform_point(&x, &y, &z, t);
        msg.dynamic_obstacle[num_obstacles].confidence = 30.0;
        msg.dynamic_obstacle[num_obstacles].x_var = x;
        msg.dynamic_obstacle[num_obstacles].y_var = y;
        msg.dynamic_obstacle[num_obstacles].xy_cov = 0;
        num_obstacles++;
      }
    }
  }*/

  msg.num_dynamic_obstacles = num_obstacles;
//  printf("%d obstacles with radar\n", num_obstacles);

}
void
integrate_sensors( dgc_velodyne_data_p velo )
{
  static bool    init = false;
  static double  last_time;
  static double  time0;
  static double  delta_s;
  int            i;

  if (!init) {
    init = true;
    if (dgc_velodyne_read_calibration( settings.velodyne_cal, velo->config )!=0) {
      fprintf( stderr, "# ERROR: could not read calibration file!\n" );
      exit(0);
    }

//    terrain_grid =
//      dgc_grid_initialize( grid_stat.resolution * 5,
//          (int) ceil(grid_stat.mapsize.x/5),
//          (int) ceil(grid_stat.mapsize.y/5),
//          sizeof(dgc_perception_map_cell_t),
//          default_terrain_cell);

    double z_resolution = grid_stat.z_resolution / CM_TO_METER_FACTOR;
    z_grid =
        dgc_grid_initialize( z_resolution,
            (int) ceil( (grid_stat.mapsize.x * grid_stat.resolution) / grid_stat.z_resolution),
            (int) ceil( (grid_stat.mapsize.y * grid_stat.resolution) / grid_stat.z_resolution),
            sizeof(short),
            &default_z_cell );
    dgc_grid_recenter_grid(z_grid, 0, 0);

    last_time = dgc_get_time();
  }

  time0 = dgc_get_time();
  delta_s = time0 - last_time;

  grid_stat.center.x =  applanix_current_pose()->smooth_x;
  grid_stat.center.y =  applanix_current_pose()->smooth_y;

  dgc_grid_recenter_grid( grid, grid_stat.center.x, grid_stat.center.y );
//  dgc_grid_recenter_grid( terrain_grid, grid_stat.center.x, grid_stat.center.y );


  /*---------------------------------------*/
  pthread_mutex_lock(&integration_mutex);
  if (settings.use_velodyne) {
    if (velo->num_scans>0 &&
        (!settings.clear_sensor_data ||
            time0-velodyne_ts<settings.max_sensor_delay)) {
      integrate_velodyne( velo, counter );
    }

  }
  pthread_mutex_unlock(&integration_mutex);
  /*---------------------------------------*/

  counter++;

  for (i=0; i<NUM_LDLRS_LASERS; i++) {
    /*---------------------------------------*/
    pthread_mutex_lock(&ldlrs_mutex[i]);
    if (settings.use_ldlrs[i]) {
      if (!settings.clear_sensor_data || time0-ldlrs_ts[i]<settings.max_sensor_delay) {
        integrate_ldlrs( &ldlrs[i], ldlrs_offset[i], counter );
      }
    }
    pthread_mutex_unlock(&ldlrs_mutex[i]);
    /*---------------------------------------*/
  }

  if (settings.extract_dynamic) {
    perception_track_frame(velo->scans->timestamp);
  }

  pthread_mutex_lock(&publish_mutex);
  perception_prep_obstacles(grid, obstacles_s, obstacles_tracked, counter);
  pthread_mutex_unlock(&publish_mutex);

  

  display_time("Integrate", time0);
  last_time = time0;
}

