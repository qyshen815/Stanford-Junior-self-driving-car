#include <dst/helper_functions.h>

using namespace std;
namespace bfs = boost::filesystem;

namespace dst
{

  ostream& operator<<(ostream& out, const cv::Vec3b& pt)
  {
    out << (int)pt[0] << " " << (int)pt[1] << " " << (int)pt[2];
    return out;
  }
  
  int sign(int x)
  {
    return (x > 0) - (x < 0);
  }
  
  std::string generateFilename(const bfs::path& dir,
			       const std::string& basename,
			       int width)
  {
    // -- Create the directory if necessary.
    ROS_ASSERT(!bfs::exists(dir) || bfs::is_directory(dir));
    if(!bfs::exists(dir))
      bfs::create_directory(dir);

    // -- Find the next number.
    int num = 0;
    bfs::directory_iterator end_itr; // default construction yields past-the-end
    for(bfs::directory_iterator itr(dir); itr != end_itr; ++itr) { 
      if(itr->leaf().substr(width+1).compare(basename) == 0)
	++num;
    }
    
    ostringstream filename;
    filename << setw(width) << setfill('0') << num << "-" << basename;
    ostringstream oss;
    oss << dir / filename.str();
    return oss.str();
  }
  
} // namespace dst
