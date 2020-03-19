#include <roadrunner.h>
#include <rndf.h>
#include <mdf.h>

using namespace dgc;

int main(int argc, char **argv)
{
  rndf_file rndf;
  mdf_file mdf;

  if(argc < 3)
    dgc_die("Error: not enough arguments.\n"
            "Usage: %s rndf-file mdf-file\n", argv[0]);
  
  if(rndf.load(argv[1]) == -1)
    fprintf(stderr, "Error: could not read RNDF file %s\n", argv[1]);
  if(rndf.save("test.txt") == -1)
    fprintf(stderr, "Error: could not save RNDF file %s\n", "test.txt");

  if(mdf.load(argv[2]) == -1)
    fprintf(stderr, "Error: could not read MDF file %s\n", argv[2]);
  if(mdf.save("mdf.txt") == -1)
    fprintf(stderr, "Error: could not save MDF file %s\n", "mdf.txt");
  
  //  rndf_print(&rndf);
  //  fprintf(stderr, "\n");
  //  mdf_print(mdf);
  return 0;
}
