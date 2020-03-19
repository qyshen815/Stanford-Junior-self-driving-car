#include <segmentation_and_tracking/scene.h>

using namespace std;
namespace bfs = boost::filesystem;
using namespace Eigen;
using boost::shared_ptr;

Sequence::Sequence(const std::string& root) :
  root_(root)
{
  assert(bfs::exists(root_));

  // -- Get the number of scenes.
  size_t num_scenes = 0;
  bfs::directory_iterator end_itr;
  for(bfs::directory_iterator itr(root); itr != end_itr; ++itr)
    if((itr->path().extension().compare(".png") == 0) | 
       (itr->path().extension().compare(".jpg") == 0))
      ++num_scenes;

  // -- Get the sorted scene names.
  size_t idx = 0;
  names_.resize(num_scenes);
  for(bfs::directory_iterator itr(root); itr != end_itr; ++itr) {
    if((itr->path().extension().compare(".png") == 0) | 
       (itr->path().extension().compare(".jpg") == 0)) {
      string filename = itr->path().filename();
      names_[idx] = filename.substr(0, filename.size() - 4);
      ++idx;
    }
  }
  sort(names_.begin(), names_.end());

  // -- Load all scenes.
  scenes_.reserve(size());
  for(size_t i = 0; i < size(); ++i)
    scenes_.push_back(shared_ptr<Scene>(loadScene(i)));
}

size_t Sequence::size() const
{
  return names_.size();
}

Scene* Sequence::loadScene(size_t idx) const
{
  assert(idx < size());
  string path = root_ + "/" + names_[idx];
  return new Scene(path);
}


shared_ptr<Scene> Sequence::getScene(size_t idx) const
{
  assert(idx < scenes_.size());
  return scenes_[idx];
}

void Sequence::saveSegmentations() const
{
  for(size_t i = 0; i < scenes_.size(); ++i) {
    if(scenes_[i]->segmentation_)
      scenes_[i]->saveSegmentation();
  }
}
