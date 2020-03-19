#include "velodyne_log.h"

using namespace std;

VelodyneLog::VelodyneLog(const std::string& cal_path, const std::string& vlf_path, dgc_transform_t velodyne_offset) :
  velodyne_file_(NULL),
  velodyne_config_(NULL)
{
  char index_filename[500];
  char vlf_filename[500];
  vlf_path.copy(vlf_filename, 500);
  vlf_filename[vlf_path.size()] = '\0'; // Really?
  cout << vlf_path << endl;
  cout << vlf_filename << endl;
  strcpy(index_filename, vlf_filename);
  strcat(index_filename, ".index.gz");
  
  if(strlen(vlf_filename) < 4 || 
     strcmp(vlf_filename + strlen(vlf_filename) - 4, ".vlf"))
    dgc_die("Error: first argument must end in .vlf\n");
  

  velodyne_file_ = dgc_velodyne_open_file(vlf_filename);
  if(velodyne_file_ == NULL)
    dgc_die("Error: Could not open velodyne file %s for reading.\n", 
	    vlf_filename);

  velodyne_index_.load(index_filename);

  // Load velodyne calibration & transform.
  char cal_filename[500];
  cal_path.copy(cal_filename, 500);
  dgc_velodyne_get_config(&velodyne_config_);
  if(dgc_velodyne_read_calibration(cal_filename, velodyne_config_) != 0) {
    fprintf(stderr, "# ERROR: could not read calibration file!\n");
    exit(0);
  }
  dgc_velodyne_integrate_offset(velodyne_offset, velodyne_config_);

  jump(1); // The 0th spin often has imcomplete data.
}

bool VelodyneLog::increment(int num)
{
  return jump(spin_num_ + num);
}

void VelodyneLog::jumpToEnd()
{
  jump(velodyne_index_.num_spins - 1);
}

bool VelodyneLog::jump(int spin_num)
{
  // -- Do bounds checking.
  bool valid = true;
  if(spin_num >= velodyne_index_.num_spins) { 
    spin_num = velodyne_index_.num_spins - 1;
    valid = false;
  }
  else if(spin_num < 0) { 
    spin_num = 0;
    valid = false;
  }

  spin_num_ = spin_num;
  
  double applanix_lat, applanix_lon, applanix_alt;
  spin_.load(velodyne_file_, velodyne_config_, &velodyne_index_, spin_num_,
	     &applanix_lat, &applanix_lon, &applanix_alt);

  return valid;
}

dgc::dgc_velodyne_index_entry VelodyneLog::getIndexEntry() const
{
  return velodyne_index_.spin[spin_num_];
}
