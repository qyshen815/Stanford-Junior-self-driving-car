#include <roadrunner.h>
#include <velocore.h>
#include <velo_support.h>
#include <velodyne_shm_interface.h>
#include "veloclient.h"

dgc_velodyne_client::dgc_velodyne_client(char *cal_filename, char *int_filename,
					 dgc_transform_t velodyne_offset)
{
  /* start with the default calibration */
  dgc_velodyne_get_config(&config);
  
  /* read the proper calibration table */
  if(dgc_velodyne_read_calibration(cal_filename, config) != 0) {
    fprintf(stderr, "# ERROR: could not read calibration file!\n");
    exit(0);
  }
  if(dgc_velodyne_read_intensity(int_filename, config) != 0) {
    fprintf(stderr, "# ERROR: could not read intensity calibration file!\n");
    exit(0);
  }

  /* save a copy of the offset transform */
  dgc_velodyne_integrate_offset(velodyne_offset, config);

  velo_interface = new VelodyneShmInterface();
  if(velo_interface->CreateClient() < 0)
    dgc_die("Error: could not connect to velodyne interface.\n");
  
  /* make shm interface ready */
  //  velodyne_shm_clear();
}

dgc_velodyne_client::~dgc_velodyne_client()
{
  free(config);
}

void dgc_velodyne_client::read_spin(dgc_velodyne_spin *spin)
{
  spin->num_scans = 
    velo_interface->ReadScans(spin->scans, MAX_NUM_VELOSCANS);
}

