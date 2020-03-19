#include <roadrunner.h>
#include <rndf.h>

using namespace dgc;

class my_rndf_data : public rndf_data {
public:
  my_rndf_data *clone(void);
  int cost;
};

#define DATA(x) (static_cast<my_rndf_data *>(x))

my_rndf_data *my_rndf_data::clone(void)
{
  my_rndf_data *d = new my_rndf_data;
  d->cost = this->cost;
  return d;
}

void add_data(rndf_file *rndf)
{
  int i, j, k;

  for(i = 0; i < rndf->num_segments(); i++)
    for(j = 0; j < rndf->segment(i)->num_lanes(); j++)
      for(k = 0; k < rndf->segment(i)->lane(j)->num_waypoints(); k++) {
	my_rndf_data *data = new my_rndf_data;
	data->cost = 0;
	rndf->segment(i)->lane(j)->waypoint(k)->data = data;
      }
  for(i = 0; i < rndf->num_zones(); i++) {
    for(j = 0; j < rndf->zone(i)->num_perimeter_points(); j++) {
      my_rndf_data *data = new my_rndf_data;
      data->cost = 0;
      rndf->zone(i)->perimeter(j)->data = data;
    }
    for(j = 0; j < rndf->zone(i)->num_spots(); j++) 
      for(k = 0; k < rndf->zone(i)->spot(j)->num_waypoints(); k++) {
	my_rndf_data *data = new my_rndf_data;
	data->cost = 0;
	rndf->zone(i)->spot(j)->waypoint(k)->data = data;
      }
  }
}

int main(int argc, char **argv)
{
  rndf_file rndf;
  int i, j, k;

  /* load the RNDF file */
  if(argc < 2)
    dgc_die("Error: not enough arguments.\n"
            "Usage: %s rndf-file\n", argv[0]);
  if(rndf.load(argv[1]) == -1)
    fprintf(stderr, "Error: could not read RNDF file %s\n", argv[1]);

  add_data(&rndf);
  
  for(i = 0; i < rndf.num_segments(); i++)
    for(j = 0; j < rndf.segment(i)->num_lanes(); j++)
      for(k = 0; k < rndf.segment(i)->lane(j)->num_waypoints(); k++) 
	fprintf(stderr, "cost = %d\n", 
		DATA(rndf.segment(i)->lane(j)->waypoint(k)->data)->cost);

  return 0;
}
