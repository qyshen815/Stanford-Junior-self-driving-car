#include <roadrunner.h>
#include <param_interface.h>
#include <applanix_interface.h>
#include <passat_constants.h>
#include <transform.h>
#include <velodyne_shm_interface.h>

using namespace dgc;

#define MAX_NUM_SCANS	10000

int main(void)
{
  dgc_velodyne_config_p	  veloconfig = NULL;
  dgc_velodyne_scan_p	  scans = NULL;
  int			  num_scans = 0;
  int scanctr = 0;
  int n, b;

  VelodyneInterface *velo_interface = new VelodyneShmInterface;
  if(velo_interface->CreateClient() < 0)
    dgc_die("Error: could not connect to velodyne interface.\n");
  
  dgc_velodyne_get_config(&veloconfig);
  scans = (dgc_velodyne_scan_p)malloc(MAX_NUM_SCANS * 
				      sizeof(dgc_velodyne_scan_t));
  dgc_test_alloc(scans);

  while(1) {
    while(velo_interface->ScanDataWaiting()) {
      num_scans = velo_interface->ReadScans(scans, MAX_NUM_SCANS);
      if(num_scans > 0) {
	printf( "#SCAN %d %d\n",
		scanctr++, num_scans * VELO_BEAMS_IN_SCAN);
	for(n = 0; n < num_scans; n++) {
	  for(b = 0; b < VELO_BEAMS_IN_SCAN; b++) {
	    printf("%f %f %f %d\n",
		   scans[n].p[b].x * 0.01 + scans[n].robot.x,
		   scans[n].p[b].y * 0.01 + scans[n].robot.y,
		   scans[n].p[b].z * 0.01 + scans[n].robot.z,
		   scans[n].p[b].intensity);
	  }
	}
      }
    }
    usleep(500);
  }
  return 0;
}
