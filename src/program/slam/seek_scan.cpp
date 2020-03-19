#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include "velo_support.h"

using namespace dgc;

/* parameters */

char *cal_filename = NULL;
dgc_transform_t velodyne_offset;

void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = {
    {"transform", "velodyne", DGC_PARAM_TRANSFORM,   &velodyne_offset, 1, NULL},
    {"velodyne", "cal_file", DGC_PARAM_FILENAME, &cal_filename, 0, NULL},
  };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
}

int main(int argc, char **argv)
{
  dgc_velodyne_file_p velodyne_file = NULL;
  dgc_velodyne_config_p velodyne_config = NULL;
  dgc_velodyne_index velodyne_index;
  dgc_velodyne_spin spin;
  double applanix_lat, applanix_lon, applanix_alt;
  IpcInterface *ipc;
  ParamInterface *pint;

  if(argc < 4)
    dgc_die("Error: not enough arguments.\n"
	    "Usage: %s velodyne-log-file velodyne-index scan-num\n", argv[0]);

  /* IPC initialization */
  ipc = new IpcStandardInterface();
  pint = new ParamInterface(ipc);
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  read_parameters(pint, argc, argv);
  delete pint;
  delete ipc;

  /* open the velodyne file */
  velodyne_file = dgc_velodyne_open_file(argv[1]);
  if(velodyne_file == NULL)
    dgc_die("Error: Could not open velodyne file %s for reading.\n", argv[1]);

  /* load the velodyne index */
  velodyne_index.load(argv[2]);

  /* load velodyne calibration & transform */
  dgc_velodyne_get_config(&velodyne_config);
  if(dgc_velodyne_read_calibration(cal_filename, velodyne_config) != 0) {
    fprintf(stderr, "# ERROR: could not read calibration file!\n");
    exit(0);
  }
  dgc_velodyne_integrate_offset(velodyne_offset, velodyne_config);

  dgc_time_code(spin.load(velodyne_file, velodyne_config, &velodyne_index, 1,
			  &applanix_lat, &applanix_lon, &applanix_alt);,"SEEK");

  dgc_time_code(spin.load(velodyne_file, velodyne_config, &velodyne_index, 2,
			  &applanix_lat, &applanix_lon, &applanix_alt);,"SEEK");

  return 0;
}
