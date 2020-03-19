#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>

using namespace dgc;

int main(int argc, char **argv)
{
  double vehicle_lat, vehicle_lon, vehicle_theta, vehicle_velocity;
  char *rndf_filename, *mdf_filename;
  int i, n;

  if(argc < 2) 
    dgc_die("Error: not enough arguments.\n"
            "Usage: %s num-vehicles\n", argv[0]);

  n = atoi(argv[1]);

  for(i = 0; i < n; i++) {
    IpcInterface *ipc = new IpcStandardInterface;
    ParamInterface *pint = new ParamInterface(ipc);

    /* connect to central */
    ipc->Connect("setup", "localhost", 1381 + i);
    
    if(i == 0) {
      pint->GetString("rndf", "rndf_file", &rndf_filename, NULL);
      pint->GetString("rndf", "mdf_file", &mdf_filename, NULL);
      printf("%s\n", rndf_filename);
      printf("%s\n", mdf_filename);
      printf("%d\n", n);
    }

    /* set simulation parameters */
    pint->GetDouble("sim", "vehicle_start_latitude", &vehicle_lat, NULL);
    pint->GetDouble("sim", "vehicle_start_longitude", &vehicle_lon, NULL);
    pint->GetDouble("sim", "vehicle_start_theta", &vehicle_theta, NULL);
    pint->GetDouble("sim", "vehicle_start_velocity", &vehicle_velocity, NULL);

    printf("%.6f %.6f %f %f\n", vehicle_lat, vehicle_lon,
	   vehicle_theta, vehicle_velocity);
        
    delete pint;
    delete ipc;
  }
  return 0;
}
