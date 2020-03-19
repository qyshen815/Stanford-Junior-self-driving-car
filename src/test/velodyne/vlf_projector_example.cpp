#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <lltransform.h>
#include <velo_support.h>

using namespace dgc;

/* parameters */

char *cal_filename = NULL;
char *int_filename = NULL;
dgc_transform_t velodyne_offset;

inline void my_spin_func(dgc_velodyne_spin *spin, 
			 dgc_velodyne_config_p config,
			 ApplanixPose *applanix_pose)
{
  double p_x, p_y, p_z, ts = 0, v, range, intensity;
  static double first_spin_ts = 0, start_ts = 0;
  static int first_spin = 1, spin_count = 0;
  int i, j, beam_num, ring_num;

  if(first_spin && spin->num_scans > 0) {
    first_spin_ts = spin->scans[0].timestamp;
    start_ts = dgc_get_time();
    first_spin = 0;
  }
  if(spin->num_scans > 0)
    ts = spin->scans[0].timestamp;
  
  for(i = 0; i < spin->num_scans; i++) 
    for(j = 0; j < VELO_BEAMS_IN_SCAN; j++) {
      beam_num = j + spin->scans[i].block * 32;
      ring_num = config->inv_beam_order[beam_num];

      if(spin->scans[i].p[j].range < 0.01)
	continue;
      
      p_x = spin->scans[i].p[j].x * 0.01 + spin->scans[i].robot.x;
      p_y = spin->scans[i].p[j].y * 0.01 + spin->scans[i].robot.y;
      p_z = spin->scans[i].p[j].z * 0.01 + spin->scans[i].robot.z;

      range = spin->scans[i].p[j].range * 0.01;
      intensity = spin->scans[i].p[j].intensity;

      v = applanix_pose->speed;
      
      // do something useful here.
    }

  spin_count++;
  fprintf(stderr, "\rProcessed %.2f s (%.1fx realtime)    ", 
	  ts - first_spin_ts, (ts - first_spin_ts) / 
	  (dgc_get_time() - start_ts));
}

void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = { 
    {"velodyne", "cal_file", DGC_PARAM_FILENAME, &cal_filename, 0, NULL},
    {"velodyne", "int_file", DGC_PARAM_FILENAME, &int_filename, 0, NULL},
    {"transform", "velodyne", DGC_PARAM_TRANSFORM, &velodyne_offset, 0, NULL}
 };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
}

int main(int argc, char **argv)
{
  IpcInterface *ipc;
  ParamInterface *pint;

  if(argc < 3)
    dgc_die("Error: not enough arguments.\n"
            "Usage: %s log-file velodyne-file\n", argv[0]);

  /* IPC initialization */
  ipc = new IpcStandardInterface();
  pint = new ParamInterface(ipc);
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  read_parameters(pint, argc, argv);
  delete pint;
  delete ipc;

  /* run through velodyne data */
  vlf_projector(argv[2], argv[1], cal_filename, int_filename, velodyne_offset, my_spin_func);
  fprintf(stderr, "\n");
  return 0;
}
