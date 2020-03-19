#include <iostream>
#include <fstream>

#include <param_interface.h>
#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <velo_support.h>
#include <lltransform.h>
#include <transform.h>

#include <octomap/octomap.h>
#include <octomap/CountingOcTree.h>


#define RESOLUTION 0.5
#define MOTION_THRESH 0.1
#define NUM_SPINS_PER_CHUNK 5

using namespace std;
using namespace dgc;
using namespace octomap;
using namespace octomath;


dgc_transform_t g_velodyne_offset;
char *g_cal_filename = NULL;

dgc_velodyne_file_p velodyne_file = NULL;
dgc_velodyne_index velodyne_index;
dgc_velodyne_config_p velodyne_config = NULL;
double applanix_lat, applanix_lon, applanix_alt;


void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = {
    {"transform", "velodyne", DGC_PARAM_TRANSFORM,   &g_velodyne_offset, 1, NULL},
    {"velodyne", "cal_file", DGC_PARAM_FILENAME, &g_cal_filename, 0, NULL},
   };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
}


void updateMapWithVirtualScan(OcTree& map, CountingOcTree& free_tree,
			      CountingOcTree& occupied_tree) { 
  // -- Update map a la OcTree::insertScanUniform.
  std::list<OcTreeVolume> free_cells;
  free_tree.getLeafNodes(free_cells);

  std::list<OcTreeVolume> occupied_cells;
  occupied_tree.getLeafNodes(occupied_cells);

  // delete free cells if cell is also measured occupied
  for (std::list<OcTreeVolume>::iterator cellit = free_cells.begin(); cellit != free_cells.end();){
    if ( occupied_tree.search(cellit->first) ) {
      cellit = free_cells.erase(cellit);
    }
    else {
      cellit++;
    }
  } // end for

    // insert data into tree  -----------------------
  for (std::list<OcTreeVolume>::iterator it = free_cells.begin(); it != free_cells.end(); it++) {
    map.updateNode(it->first, false);
  }
  for (std::list<OcTreeVolume>::iterator it = occupied_cells.begin(); it != occupied_cells.end(); it++) {
    map.updateNode(it->first, true);
  }
}

// Integrates several spins all at once into the map, ignoring the passing of rays through cells with at least one hit.
// This should avoid grazing effects.
void processChunk(int start, int num_spins, OcTree& map) {
  dgc_velodyne_spin spin;
  CountingOcTree free_tree(map.getResolution());
  CountingOcTree occupied_tree(map.getResolution());
  std::vector<point3d> ray;

  for(int i = start; i < start + num_spins; ++i) {
    
    spin.load(velodyne_file, velodyne_config, &velodyne_index, i,
	      &applanix_lat, &applanix_lon, &applanix_alt);
    
    for(int j = 0; j < spin.num_scans; j++) {
      
      // -- Get the velodyne center for this scan.
      dgc_transform_t rotation;
      dgc_transform_t id;
      dgc_transform_identity(id);
      dgc_transform_rpy(rotation, id,
			spin.scans[j].robot.roll,
			spin.scans[j].robot.pitch,
			spin.scans[j].robot.yaw);
	
      double laser_x = velodyne_config->offset[0][3];
      double laser_y = velodyne_config->offset[1][3];
      double laser_z = velodyne_config->offset[2][3];
      dgc_transform_point(&laser_x, &laser_y, &laser_z, rotation);
	
      laser_x += spin.scans[j].robot.x;
      laser_y += spin.scans[j].robot.y;
      laser_z += spin.scans[j].robot.z;

      // -- Update the map for all points in this scan.
      for(int k = 0; k < VELO_BEAMS_IN_SCAN; k++) {
	if(spin.scans[j].p[k].range < 0.01)
	  continue;

	point3d origin(laser_x, laser_y, laser_z);
	point3d endpoint(spin.scans[j].p[k].x / 100. + spin.scans[j].robot.x,
			 spin.scans[j].p[k].y / 100. + spin.scans[j].robot.y,
			 spin.scans[j].p[k].z / 100. + spin.scans[j].robot.z);

	if(map.computeRay(origin, endpoint, ray)){
	  for(std::vector<point3d>::iterator it=ray.begin(); it != ray.end(); it++) {
	    free_tree.updateNode(*it);
	  }
	}
	occupied_tree.updateNode(endpoint);

	//TODO: If max range, update traced cells.
		
      }
    }
  }

  updateMapWithVirtualScan(map, free_tree, occupied_tree);
}

int main(int argc, char** argv) {
  
  // -- Load vlf and log.gz.
  char vlf_filename[300], index_filename[300];

  IpcInterface *ipc;
  ParamInterface *pint;

  if(argc < 2)
    dgc_die("Error: not enough arguments.\n"
	    "Usage: %s vlf-file\n", argv[0]);

  /* IPC initialization */
  ipc = new IpcStandardInterface();
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  pint = new ParamInterface(ipc);
  read_parameters(pint, argc, argv);
  delete ipc;

  strcpy(vlf_filename, argv[1]);
  if(strlen(vlf_filename) < 4 || 
     strcmp(vlf_filename + strlen(vlf_filename) - 4, ".vlf"))
    dgc_die("Error: first argument must end in .vlf\n");

  strcpy(index_filename, argv[1]);
  strcat(index_filename, ".index.gz");

  /* open velodyne file */
  cout << "Loading vlf file " << vlf_filename << endl;
  velodyne_file = dgc_velodyne_open_file(vlf_filename);
  if(velodyne_file == NULL)
    dgc_die("Error: Could not open velodyne file %s for reading.\n", 
	    vlf_filename);

  /* load the velodyne index */
  cout << "Loading index file " << index_filename << endl;
  velodyne_index.load(index_filename);

  /* load velodyne calibration & transform */
  dgc_velodyne_get_config(&velodyne_config);
  if(dgc_velodyne_read_calibration(g_cal_filename, velodyne_config) != 0) {
    fprintf(stderr, "# ERROR: could not read calibration file!\n");
    exit(0);
  }
  dgc_velodyne_integrate_offset(g_velodyne_offset, velodyne_config);
  
  // -- Generate the map.
  OcTree map(RESOLUTION);

  int num_spins_per_chunk = NUM_SPINS_PER_CHUNK;
  //int max_spin_num = velodyne_index.num_spins;
  int max_spin_num = 1100;
  for(int i = 960; i < max_spin_num; i+=num_spins_per_chunk) {
    if(i + num_spins_per_chunk >= max_spin_num)
      break;

    cout << "Processing spins " << i << " to " << i + num_spins_per_chunk - 1 << " / " << max_spin_num
	 << ", chunk " << i / num_spins_per_chunk << " / " << max_spin_num / num_spins_per_chunk << "... "; cout.flush();

    // Check for movement.
    double x = velodyne_index.spin[i].pose[0].smooth_x - velodyne_index.spin[i+num_spins_per_chunk].pose[0].smooth_x;
    double y = velodyne_index.spin[i].pose[0].smooth_y - velodyne_index.spin[i+num_spins_per_chunk].pose[0].smooth_y;
    double z = velodyne_index.spin[i].pose[0].smooth_z - velodyne_index.spin[i+num_spins_per_chunk].pose[0].smooth_z;
    if(sqrt(x*x + y*y + z*z) < MOTION_THRESH) {
      cout << " no motion!  Skipping." << endl;
      continue;
    }

    clock_t start = clock();
    processChunk(i, num_spins_per_chunk, map);
    float num_sec = (float)(clock() - start) / (float)(CLOCKS_PER_SEC);
    cout << " took " << num_sec << " seconds, "
	 << num_sec * (float)(max_spin_num - i) / (float)num_spins_per_chunk / 60.
	 << " minutes remaining." <<  endl;
  }

  // -- Save the map.
  map.writeBinary("map.bt");

  
  return 0;
}
