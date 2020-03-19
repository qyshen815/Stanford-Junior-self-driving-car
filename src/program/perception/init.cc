#include "perception.h"

using namespace dgc;
using namespace vlr;

dgc_perception_map_cells_p       obstacles_s;

std::vector<std::tr1::shared_ptr<TrackedObstacle> > obstacles_predicted;
std::vector<std::tr1::shared_ptr<TrackedObstacle> > obstacles_tracked;
std::vector<std::tr1::shared_ptr<Obstacle> >        obstacles_segmented;

PerceptionScan                   virtual_scan[NUM_VIRTUAL_SCANS];

void 
perception_init( void )
{
  fprintf( stderr, "# INFO: allocate structures ... " );

  obstacles_s = 
    (dgc_perception_map_cells_p) malloc(sizeof(dgc_perception_map_cells_t));
  obstacles_s->num = 0;
  obstacles_s->cell = 
    (dgc_perception_map_cell_p *) malloc( MAX_NUM_POINTS *
					  sizeof(dgc_perception_map_cell_p) );
  obstacles_s_publish.cell = NULL;
  fprintf( stderr, "done\n" );
}




