#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>

using namespace dgc;

int main(int argc, char **argv)
{
  FILE *fp;
  double vehicle_lat, vehicle_lon, vehicle_theta, vehicle_velocity;
  int i, n, count = 0;
  char rndf_filename[1000], mdf_filename[1000];

  if(argc < 2) 
    dgc_die("Error: not enough arguments.\n"
	    "Usage: %s simulation-config-file\n", argv[0]);

  dgc_randomize(&argc, &argv);
  fp = fopen(argv[1], "r");
  if(fp == NULL)
    dgc_die("Error: could not open file %s for reading.\n", argv[1]);

  if(fscanf(fp, "%s", rndf_filename) != 1)
    dgc_die("Error: bad file format in %s.", argv[1]);
  if(fscanf(fp, "%s", mdf_filename) != 1)
    dgc_die("Error: bad file format in %s.", argv[1]);
  if(fscanf(fp, "%d\n", &n) != 1)
    dgc_die("Error: bad file format in %s.", argv[1]);

  for(i = 0; i < n; i++) {
    IpcInterface *ipc = new IpcStandardInterface;
    ParamInterface *pint = new ParamInterface(ipc);

      /* read vehicle state */
    if(fscanf(fp, "%lf %lf %lf %lf", &vehicle_lat, &vehicle_lon, 
	     &vehicle_theta, &vehicle_velocity) != 4)
      dgc_die("Error: bad file format in %s.", argv[1]);
      
    /* connect to central */
    
    ipc->Connect("setup", "localhost", 1381 + count);
    
    count++;
    
    pint->SetString("rndf", "rndf_file", rndf_filename, NULL);
    pint->SetString("rndf", "mdf_file", mdf_filename, NULL);
    
    /* set simulation parameters */
    pint->SetDouble("sim", "vehicle_start_latitude", vehicle_lat, NULL);
    pint->SetDouble("sim", "vehicle_start_longitude", vehicle_lon, NULL);
    pint->SetDouble("sim", "vehicle_start_theta", vehicle_theta, NULL);
    pint->SetDouble("sim", "vehicle_start_velocity", vehicle_velocity, NULL);

    pint->SetDouble("controller", "velocity_setpoint_mph", 
		    dgc_uniform_random(20, 30), NULL);
    
    /* move onto next central */
    delete pint;
    delete ipc;
  }
  fclose(fp);
  return 0;
}
