#include <roadrunner.h>
#include <dirent.h>
#include <ipc_std_interface.h>
#include <param_interface.h>

using namespace dgc;

char *image_root = NULL;

void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = {
    {"imagery", "root", DGC_PARAM_FILENAME, &image_root, 0, NULL},
  };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
}

int is_bmp_image(de_const_ struct dirent *d)
{
  if(strlen(d->d_name) < 4)
    return 0;
  if(strcmp(d->d_name + strlen(d->d_name) - 4, ".bmp") == 0)
    return 1;
  return 0;
}

int main(int argc, char **argv)
{
  IpcInterface *ipc;
  ParamInterface *pint;
  char dirname[500];
  char filename[500];
  char cmd[1000];

  struct dirent **namelist;
  int n;

  ipc = new IpcStandardInterface();
  pint = new ParamInterface(ipc);
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  read_parameters(pint, argc, argv);

  strcpy(dirname, image_root);
  strcat(dirname, "/laser/");

  n = scandir(dirname, &namelist, is_bmp_image, alphasort);
  if(n < 0)
    dgc_die("Error: could not get file list\n");
  else {
    while(n--) {
      fprintf(stderr, "Converting %s\n", namelist[n]->d_name);

      strcpy(filename, namelist[n]->d_name);
      strcpy(filename + strlen(filename) - 4, ".gif");
      
      sprintf(cmd, "convert %s/laser/%s %s/laser/%s", image_root, 
              namelist[n]->d_name, image_root, filename);
      if(system(cmd) == -1)
        dgc_error("Failed to run command %s", cmd);

      sprintf(cmd, "rm %s/laser/%s", image_root, namelist[n]->d_name);
      if(system(cmd) == -1)
        dgc_error("Failed to run command %s", cmd);
      free(namelist[n]);
    }
  }

  return 0;
}
