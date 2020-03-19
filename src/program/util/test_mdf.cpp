#include <roadrunner.h>
#include <rndf.h>
#include <mdf.h>
#include <ipc_std_interface.h>
#include <param_interface.h>

using namespace dgc;

char *rndf_filename = NULL;
char *mdf_filename = NULL;

rndf_file *rndf = NULL;
mdf_file *mdf = NULL;

void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param param[] = {
    {"rndf", "rndf_file", DGC_PARAM_FILENAME, &rndf_filename, 0, NULL},
    {"rndf", "mdf_file", DGC_PARAM_FILENAME, &mdf_filename, 0, NULL},
  };
  pint->InstallParams(argc, argv, param, sizeof(param) / sizeof(param[0]));
}

int main(int argc, char **argv)
{
  IpcInterface *ipc;
  ParamInterface *pint;

  ipc = new IpcStandardInterface();
  pint = new ParamInterface(ipc);
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  read_parameters(pint, argc, argv);

  /* read RNDF and MDF files */
  rndf = new rndf_file;
  if(rndf->load(rndf_filename) < 0)
    dgc_die("Error: could not read RNDF file %s\n", rndf_filename);

  fprintf(stderr, "Successfully loaded RNDF %s\n", rndf_filename);

  rndf->build_checkpoint_map();
  mdf = new mdf_file;
  if(mdf->load(mdf_filename) < 0)
    dgc_die("Error: could not read MDF file %s\n", mdf_filename);

  fprintf(stderr, "Successfully loaded MDF %s\n", mdf_filename);

  return 0;
}
