#include <segmentation_and_tracking/scene.h>

using namespace std;
namespace bfs = boost::filesystem;
using namespace Eigen;

TrackedObject::TrackedObject()
{
}

void TrackedObject::serialize(std::ostream& out) const
{
  out.write((const char*)&id_, sizeof(int));
  int buf = indices_.size();
  out.write((const char*)&buf, sizeof(int));
  for(size_t i = 0; i < indices_.size(); ++i)
    out.write((const char*)&indices_[i], sizeof(int));
}

void TrackedObject::deserialize(std::istream& in)
{
  in.read((char*)&id_, sizeof(int));
  int num_indices;
  in.read((char*)&num_indices, sizeof(int));
  indices_.clear();
  indices_.resize(num_indices);
  for(int i = 0; i < num_indices; ++i)
    in.read((char*)&indices_[i], sizeof(int));
}
