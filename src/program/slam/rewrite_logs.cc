#include <roadrunner.h>
#include <localize_interface.h>
#include <applanix_interface.h>
#include "slam_inputs.h"
#include <vector>

using namespace dgc;
using namespace std;

typedef struct {
  double smooth_dx, smooth_dy, smooth_dz;
  double dlat, dlon, dalt;
  double timestamp;
} PoseOffset;

void BuildOffsetList(dgc_velodyne_index *old_index, 
		     dgc_velodyne_index *new_index,
		     vector <PoseOffset> &offset)
{
  int i, j, mark = 0, max_offsets = 0;

  for(i = 0; i < old_index->num_spins; i++)
    max_offsets += old_index->spin[i].num_poses;

  offset.resize(max_offsets);

  for(i = 0; i < old_index->num_spins; i++)
    for(j = 0; j < old_index->spin[i].num_poses; j++) {
      if(mark == 0 || old_index->spin[i].pose[j].timestamp !=
	 offset[mark - 1].timestamp) {
	offset[mark].smooth_dx = 
	  new_index->spin[i].pose[j].smooth_x -
	  old_index->spin[i].pose[j].smooth_x;
	offset[mark].smooth_dy = 
	  new_index->spin[i].pose[j].smooth_y -
	  old_index->spin[i].pose[j].smooth_y;
	offset[mark].smooth_dz = 
	  new_index->spin[i].pose[j].smooth_z -
	  old_index->spin[i].pose[j].smooth_z;

	offset[mark].dlat = 
	  new_index->spin[i].pose[j].latitude -
	  old_index->spin[i].pose[j].latitude;
	offset[mark].dlon = 
	  new_index->spin[i].pose[j].longitude -
	  old_index->spin[i].pose[j].longitude;
	offset[mark].dalt = 
	  new_index->spin[i].pose[j].altitude -
	  old_index->spin[i].pose[j].altitude;

	offset[mark].timestamp = old_index->spin[i].pose[j].timestamp;
	mark++;
      }
    }
}

void RewriteLogfile(vector <PoseOffset> &offset, char *old_filename, 
		    char *new_filename)
{
  int count = 0, current_offset = 0, first = 1, percent, last_percent = -1;
  double localize_x_offset = 0, localize_y_offset = 0, dt, utm_x, utm_y;
  ApplanixPose applanix_pose;
  LocalizePose localize_pose;
  LineBuffer *line_buffer = NULL;
  char *line = NULL, *s, utmzone[10];
  dgc_FILE *old_fp, *new_fp;
  bool done = false;

  old_fp = dgc_fopen(old_filename, "r");
  if(old_fp == NULL)
    dgc_die("Error: could not open file %s for reading.\n", old_filename);

  new_fp = dgc_fopen(new_filename, "w");
  if(new_fp == NULL)
    dgc_die("Error: could not open file %s for writing.\n", new_filename);

  current_offset = 0;

  line_buffer = new LineBuffer;
  do {
    /* read a complete line */
    line = line_buffer->ReadLine(old_fp);
    if(line != NULL) {
      if(strncmp(line, "LOCALIZE_POSE2", 14) == 0) {
	s = StringV2ToLocalizePose(dgc_next_word(line), &localize_pose);
	if(count > 0) {
	  localize_pose.source = DGC_LOCALIZE_SOURCE_LASER;
	  localize_pose.x_offset = localize_x_offset;
	  localize_pose.y_offset = localize_y_offset;
	  LocalizePoseWrite(&localize_pose, atof(s), new_fp);
	}
      }
      else if(strncmp(line, "APPLANIX_POSE_V2", 16) == 0) {
	s = StringV2ToApplanixPose(dgc_next_word(line), &applanix_pose);
	
	while(!done && fabs(applanix_pose.timestamp - 
			    offset[current_offset + 1].timestamp) <
	      fabs(applanix_pose.timestamp - 
		   offset[current_offset].timestamp)) {
	  current_offset++;
	  if(current_offset + 1 >= (int)offset.size())
	    done = true;
	}

	percent = (int)rint(current_offset / (double)offset.size() * 100);
	if(percent != last_percent) {
	  fprintf(stderr, "\rRewriting logfile (%d%%)   ", percent);
	  last_percent = percent;
	}
	
	dt = applanix_pose.timestamp - offset[current_offset].timestamp;

	applanix_pose.smooth_x += offset[current_offset].smooth_dx;
	applanix_pose.smooth_y += offset[current_offset].smooth_dy;
	applanix_pose.smooth_z += offset[current_offset].smooth_dz;

	applanix_pose.latitude += offset[current_offset].dlat;
	applanix_pose.longitude += offset[current_offset].dlon;
	applanix_pose.altitude += offset[current_offset].dalt;

	if(first) {
	  vlr::latLongToUtm(applanix_pose.latitude, applanix_pose.longitude,
		      &utm_x, &utm_y, utmzone);
	  localize_x_offset = utm_x - applanix_pose.smooth_x;
	  localize_y_offset = utm_y - applanix_pose.smooth_y;
	  first = 0;
	}
	
	ApplanixPoseWrite(&applanix_pose, atof(s), new_fp);
	count++;
      }
      else
	dgc_fprintf(new_fp, "%s\n", line);
    }
  } while(line != NULL);
  fprintf(stderr, "\rRewriting logfile (%d%%)   \n", 100);
  dgc_fclose(old_fp);
  dgc_fclose(new_fp);
}

void RewriteLogfiles(SlamInputs *inputs)
{
  vector <PoseOffset> offset;
  char *new_filename;
  int i;

  for (i = 0; i < inputs->num_logs(); i++)
    if (inputs->log(i)->optimize()) {
      offset.clear();
      BuildOffsetList(inputs->log(i)->index(), 
		      inputs->log(i)->corrected_index(), offset);

      new_filename = new char[strlen(inputs->log(i)->log_filename()) + 10];
      strcpy(new_filename, inputs->log(i)->log_filename());

      if (strcmp(new_filename + strlen(new_filename) - 4, ".log") == 0) 
	strcpy(new_filename + strlen(new_filename) - 4, ".fixed.log");
      else if (strcmp(new_filename + 
		      strlen(new_filename) - 7, ".log.gz") == 0) 
	strcpy(new_filename + strlen(new_filename) - 7, ".fixed.log.gz");
      else
	dgc_die("Input logfile did not end in .log or .log.gz."
		"  This shouldn't happen\n");

      RewriteLogfile(offset, inputs->log(i)->log_filename(), new_filename); 
      delete [] new_filename;
    }
}
