#include <roadrunner.h>
#include "heuristic.h"

int main(int argc, char **argv)
{
  heuristic_table *htable = NULL;

  if(argc < 8)
    dgc_die("Error: not enough arguments.\n"
	    "Usage: %s heuristic-file min_x min_y xy-res x_size y_size theta_size\n",
	    argv[0]);

  htable = new heuristic_table(atof(argv[2]), atof(argv[3]),
			       atof(argv[4]), atoi(argv[5]), 
			       atoi(argv[6]), atoi(argv[7]));

  if(dgc_file_exists(argv[1]))
    dgc_die("Error: file %s already exists.\n", argv[1]);
  htable->save(argv[1]);
  return 0;
}
