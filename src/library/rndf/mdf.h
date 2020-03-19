#ifndef DGC_MDF_H
#define DGC_MDF_H

#include <vector>
#include <string>
#include "rndf.h"

namespace dgc {

class mdf_speed_limit {
 public:
  int id;
  float min_speed, max_speed;
};

class mdf_file {
 public:
  int load(char *filename);
  int save(char *filename) const;

  std::string filename(void) const { return filename_; }
  void filename(std::string filename) { filename_ = filename; }
  std::string rndf_filename(void) const { return rndf_filename_; }
  void rndf_filename(std::string rndf_filename) { rndf_filename_ = rndf_filename; }
  std::string format_version(void) const { return format_version_; }
  void format_version(std::string format_version) { format_version_ = format_version; }
  std::string creation_date(void) const { return creation_date_; }
  void creation_date(std::string creation_date) { creation_date_ = creation_date; }

  int num_goals(void) const { return (signed)goal_.size(); }
  int goal(int i) const;

  int num_speed_limits(void) const { return (signed)speed_limit_.size(); }
  mdf_speed_limit *speed_limit(int i) const;

  inline double max_segment_speed_limit(rndf_file *rndf, int id) const;
  inline double min_segment_speed_limit(rndf_file *rndf, int id) const;
  inline double max_zone_speed_limit(rndf_file *rndf, int id) const;
  inline double min_zone_speed_limit(rndf_file *rndf, int id) const;

 private:
  std::string filename_;
  std::string rndf_filename_;
  std::string format_version_;
  std::string creation_date_;

  std::vector <int> goal_;
  std::vector <mdf_speed_limit *> speed_limit_;
};

void mdf_print(mdf_file *mdf);

/* inline functions below */

inline int mdf_file::goal(int i) const
{
  if(i < 0 || i >= num_goals()) 
    dgc_die("Error: mdf_file::goal : index out of range\n"); 
  return goal_[i];
}

inline mdf_speed_limit *mdf_file::speed_limit(int i) const
{
  if(i < 0 || i >= num_speed_limits()) 
    dgc_die("Error: mdf_file::speed_limit : index out of range\n"); 
  return speed_limit_[i];
}

inline double mdf_file::max_segment_speed_limit(rndf_file *rndf, int id) const 
{
  int i;
  
  if(id < 0 || id >= rndf->num_segments())
    dgc_die("Error: mdf_file::min_segment_speed_limit : index out of range.\n");
  for(i = 0; i < num_speed_limits(); i++)
    if(speed_limit(i)->id == id)
      return speed_limit(i)->max_speed;
  return dgc_mph2ms(30.0);
}

inline double mdf_file::min_segment_speed_limit(rndf_file *rndf, int id) const 
{
  int i;
  
  if(id < 0 || id >= rndf->num_segments())
    dgc_die("Error: mdf_file::min_segment_speed_limit : index out of range.\n");
  for(i = 0; i < num_speed_limits(); i++)
    if(speed_limit(i)->id == id)
      return speed_limit(i)->min_speed;
  return dgc_mph2ms(30.0);
}

inline double mdf_file::max_zone_speed_limit(rndf_file *rndf, int id) const 
{
  int i;
  
  if(id < 0 || id >= rndf->num_zones())
    dgc_die("Error: mdf_file::max_zone_speed_limit : index out of range.\n");
  for(i = 0; i < num_speed_limits(); i++)
    if(speed_limit(i)->id == id + rndf->num_segments())
      return speed_limit(i)->max_speed;
  return dgc_mph2ms(30.0);
}

inline double mdf_file::min_zone_speed_limit(rndf_file *rndf, int id) const 
{
  int i;
  
  if(id < 0 || id >= rndf->num_zones())
    dgc_die("Error: mdf_file::max_zone_speed_limit : index out of range.\n");
  for(i = 0; i < num_speed_limits(); i++)
    if(speed_limit(i)->id == id + rndf->num_segments())
      return speed_limit(i)->min_speed;
  return dgc_mph2ms(30.0);
}

}

#endif
