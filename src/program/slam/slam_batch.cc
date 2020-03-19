#include <roadrunner.h>
#include <roadrunner.h>
#include <imagery.h>
#include <velocore.h>
#include <velo_support.h>
#include <grid.h>
#include <lltransform.h>
#include <textures.h>
#include <kdtree.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <vector>
#include <ANN.h>

#include "slam_inputs.h"
#include "spins.h"
#include "intensitycal.h"
#include "new_scanmatch.h"
#include "match_thread.h"
#include "overlap.h"
#include "correctindex.h"
#include "rewrite_logs.h"

#define       NUM_THREADS         2

using namespace dgc;
using namespace vlr;

/* params */

char *imagery_root;
char *rndf_filename;
dgc_transform_t velodyne_offset;
char *cal_filename;
char *intensity_cal_filename;

/* application variables */

char *input_filename = NULL;
SlamInputs inputs;
SlamOrigin origin;
MatchList matches;
int current_match = -1;
SelectedSpin spin1, spin2;
ScanMatchThreadManager match_manager;

void SaveState(char *filename)
{
  int i, j, k;
  FILE *fp;

  fp = fopen(filename, "w");
  if (fp == NULL)
    dgc_die("Could not open file %s for reading.\n", filename);

  fprintf(fp, "%d\n", inputs.num_logs());
  for (i = 0; i < inputs.num_logs(); i++)
    fprintf(fp, "%s\t%s\t%d\n", inputs.log(i)->log_filename(),
	    inputs.log(i)->vlf_filename(), 1);

  fprintf(fp, "%d\n", matches.num_matches());
  for (i = 0; i < matches.num_matches(); i++) {
    fprintf(fp, "%d %d %d %d ", 
	    matches.match(i)->tnum1, matches.match(i)->snum1,
	    matches.match(i)->tnum2, matches.match(i)->snum2);
    for (j = 0; j < 4; j++)
      for(k = 0; k < 4; k++)
	fprintf(fp, "%f ", matches.match(i)->offset[j][k]);
    fprintf(fp, "%d\n", matches.match(i)->optimized);
  }
  fclose(fp);
}

void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = {
    {"imagery", "root", DGC_PARAM_FILENAME, &imagery_root, 0, NULL},
    {"rndf", "rndf_file", DGC_PARAM_FILENAME, &rndf_filename, 0, NULL},
    {"transform", "velodyne", DGC_PARAM_TRANSFORM,   &velodyne_offset, 1, NULL},
    {"velodyne", "cal_file", DGC_PARAM_FILENAME, &cal_filename, 0, NULL},
    {"velodyne", "intensity_cal", DGC_PARAM_FILENAME, &intensity_cal_filename, 0, NULL},
  };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
}

int main(int argc, char **argv)
{
  dgc_velodyne_config_p velodyne_config = NULL;
  IntensityCalibration intensity_calibration;
  double current_time, last_save = 0;
  int i;

  IpcInterface *ipc = new IpcStandardInterface();
  if (ipc->Connect(argv[0]) < 0)
    dgc_die("Could not connect to IPC network.");
  ParamInterface *pint = new ParamInterface(ipc);
  read_parameters(pint, argc, argv);
  delete pint;
  delete ipc;

  /* load velodyne calibration & transform */
  dgc_velodyne_get_config(&velodyne_config);
  if(dgc_velodyne_read_calibration(cal_filename, velodyne_config) != 0) {
    fprintf(stderr, "# ERROR: could not read calibration file!\n");
    exit(0);
  }
  dgc_velodyne_integrate_offset(velodyne_offset, velodyne_config);

  if (intensity_calibration.Load(intensity_cal_filename) < 0)
    dgc_die("Could not load intensity calibration %s\n", 
	    intensity_cal_filename);

  if (argc == 2 && strcmp(argv[1] + strlen(argv[1]) - 4, ".txt") == 0) {
    input_filename = strdup(argv[1]);
    dgc_info("Loading state from %s\n", input_filename);
    inputs.LoadFromFile(input_filename, &matches, velodyne_config, 
			&intensity_calibration);
  } else {
    inputs.DiscoverInputFiles(argc, argv, velodyne_config, 
			      &intensity_calibration);
    if (dgc_file_exists("match.txt"))
      dgc_die("match.txt already exists.\n");
    input_filename = strdup("match.txt");
    SaveState(input_filename);
    FindOverlap(&inputs, &matches);
    fprintf(stderr, "Found %d matches.\n", matches.num_matches());
  }

  match_manager.StartThreads(NUM_THREADS);

  for (i = 0; i < matches.num_matches(); i++)
    if (!matches.match(i)->optimized)
      match_manager.AssignMatch(&inputs, matches.match(i), i);

  last_save = 0;
  do {
    current_time = dgc_get_time();
    if (current_time - last_save > 5.0) {
      SaveState(input_filename);
      last_save = current_time;
    }
      
    usleep(200000);
  } while (match_manager.Busy());
  SaveState(input_filename);

  SLAMCorrectIndexes(&inputs, &matches);
  for (i = 0; i < inputs.num_logs(); i++) 
    inputs.log(i)->GenerateCorrectedPathDL(&origin);
  
  RewriteLogfiles(&inputs);
   
  return 0;
}
