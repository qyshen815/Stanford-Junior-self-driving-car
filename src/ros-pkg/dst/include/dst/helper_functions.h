#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H

#include <boost/filesystem.hpp>
#include <ros/assert.h>
#include <opencv2/core/core.hpp>

namespace dst
{

  std::ostream& operator<<(std::ostream& out, const cv::Vec3b& pt);
  
  int sign(int x);

  std::string generateFilename(const boost::filesystem::path& dir,
			       const std::string& basename,
			       int width = 4);
  
}

#endif // HELPER_FUNCTIONS_H
