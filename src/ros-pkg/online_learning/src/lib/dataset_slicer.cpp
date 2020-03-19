#include <online_learning/online_learning.h>

using namespace std;

namespace odontomachus
{

  void DatasetSlicer::slice(Dataset::Ptr data, int num_projections) const 
  {
    NameMapping2 dmap = sliceDescriptorMap(data->descriptor_map_, num_projections);
    data->applyNameMapping(dmap, 1);
    ROS_ASSERT((size_t)data->descriptors_.rows() == dmap.size());
  }
  
  NameMapping2 DatasetSlicer::sliceDescriptorMap(const NameMapping2& orig, int np) const
  {
    ROS_ASSERT(orig.getOffset() == 0);
    
    // -- Get unique descriptor space names (i.e. before hashing occurred).
    set<string> uniq;
    vector<string> ordered;
    ordered.reserve(orig.size());
    for(size_t i = 0; i < orig.size(); ++i) {
      string name = orig.toName(i);
      string dspace_name = name.substr(0, name.find_first_of(":"));
      if(uniq.count(dspace_name) == 0) { 
	uniq.insert(dspace_name);
	ordered.push_back(dspace_name);
      }
    }

    // -- Add the first np hash names for each unique descriptor space.
    NameMapping2 dmap;
    for(size_t i = 0; i < ordered.size(); ++i) { 
      string dspace_name = ordered[i];
      vector<string> hash_names;
      for(size_t j = 0; j < orig.size(); ++j) {
	string hash_name = orig.toName(j);
	if(dspace_name.compare(hash_name.substr(0, dspace_name.size())) == 0)
	  hash_names.push_back(hash_name);
      }

      ROS_ASSERT((size_t)np <= hash_names.size());
      for(int j = 0; j < np; ++j)
	dmap.addName(hash_names[j]);
    }

    if(dmap.size() == orig.size())
      ROS_ASSERT(dmap == orig);
    
    return dmap;
  }

} // namespace odontomachus
