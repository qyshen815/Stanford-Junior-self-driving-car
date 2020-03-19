#include <multibooster/multibooster.h>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
  MultiBoosterDataset mbd(argv[1]);
  cout << mbd.status() << endl;

  size_t num_descr = mbd.objs_[0]->descriptors_.size();
  set<Object*> objects_perm;
  for(size_t i = 0; i < mbd.objs_.size(); ++i) {
    assert(mbd.objs_[i]->descriptors_.size() == num_descr);
    objects_perm.insert(mbd.objs_[i]);
  }


  for(size_t i = 0; i < num_descr; ++i) {
    int num_unique = 0;
    set<Object*>::iterator it;
    set<Object*> objects = objects_perm;
    while(!objects.empty()) {
      ++num_unique;
      Object* active = *objects.begin();
      objects.erase(active);
      vector<Object*> to_delete;
      for(it = objects.begin(); it != objects.end(); ++it) {
	if(fastEucSquared(*(*it)->descriptors_[i].vector,
			  *active->descriptors_[i].vector,
			  (*it)->descriptors_[i].length_squared,
			  active->descriptors_[i].length_squared) < 1e-4) {
	  to_delete.push_back(*it);
	}
      }
      for(size_t i = 0; i < to_delete.size(); ++i) {
	objects.erase(to_delete[i]);
      }
    }
    cout << "Descriptor " << mbd.feature_map_.toName(i) << " has " << num_unique << " unique descriptors, out of " << mbd.objs_.size() << " total." << endl;
  }
    
  return 0;
}
