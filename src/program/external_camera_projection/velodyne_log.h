#ifndef VELODYNE_LOG_H
#define VELODYNE_LOG_H

#include <roadrunner.h>
#include <transform.h>
#include <velo_support.h>
#include <string>
#include <iostream>

class VelodyneLog
{
public:
  dgc::dgc_velodyne_index velodyne_index_;
  dgc::dgc_velodyne_spin spin_;
  int spin_num_;
  
  VelodyneLog(const std::string& cal_path, const std::string& vlf_path, dgc_transform_t velodyne_offset);
  //! If incrementing by num will cause you to run off the end, this will stop at the end and return false.
  bool increment(int num);
  //! If you try to jump beyond the end of the log, this will go to the end and return false.
  bool jump(int spin_num);
  void jumpToEnd();
  dgc::dgc_velodyne_index_entry getIndexEntry() const;
  
private:
  dgc_velodyne_file_p velodyne_file_;
  dgc_velodyne_config_p velodyne_config_;
};

#endif // VELODYNE_LOG_H
