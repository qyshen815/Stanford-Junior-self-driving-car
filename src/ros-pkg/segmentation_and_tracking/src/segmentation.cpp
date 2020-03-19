#include <segmentation_and_tracking/scene.h>

using namespace std;
namespace bfs = boost::filesystem;
using namespace Eigen;


Segmentation::Segmentation()
{
}

Segmentation::Segmentation(const string& path)
{
  assert(bfs::exists(path));
  ifstream file(path.c_str());
  deserialize(file);
  file.close();
}

void Segmentation::save(const std::string& path) const
{
  ofstream file(path.c_str());
  serialize(file);
  file.close();
}

void Segmentation::deserialize(std::istream& in)
{
  size_t num_tracked_objects;
  in.read((char*)&num_tracked_objects, sizeof(size_t));
  cout << num_tracked_objects << " tracked objects" << endl;
  
  tracked_objects_.resize(num_tracked_objects);
  for(size_t i = 0; i < tracked_objects_.size(); ++i)
    tracked_objects_[i].deserialize(in);
}

void Segmentation::serialize(std::ostream& out) const
{
  size_t num_tracked_objects = tracked_objects_.size();
  out.write((const char*)&num_tracked_objects, sizeof(size_t));
  for(size_t i = 0; i < tracked_objects_.size(); ++i)
    tracked_objects_[i].serialize(out);
}
