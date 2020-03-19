#include <track_manager_cached_descriptors.h>

#define NUM_THREADS getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1

using namespace std;
using boost::shared_ptr;

namespace track_manager { 

  TrackManagerCachedDescriptors::TrackManagerCachedDescriptors(const std::string& path)
  {
    assert(path.substr(path.length() - 7).compare(".tm.mbd") == 0);
    ifstream file;
    file.open(path.c_str());
    deserialize(file);
    file.close();
  }

  TrackManagerCachedDescriptors::TrackManagerCachedDescriptors(const TrackManager& tm,
							       const NameMapping& class_map) :
    tracks_(tm.tracks_.size()),
    class_map_(class_map)
  {
    DescriptorPipeline dp(NUM_THREADS);
    descriptor_map_ = dp.getDescriptorMap();
    
    for(size_t i = 0; i < tm.tracks_.size(); ++i) {
      tracks_[i] = shared_ptr<TrackCachedDescriptors>(new TrackCachedDescriptors(this, dp.computeDescriptors(*tm.tracks_[i])));
      
      // -- Set the label for each object.
      int label;
      if(tm.tracks_[i]->label_.compare("background") == 0)
	label = -1;
      else if(tm.tracks_[i]->label_.compare("unlabeled") == 0)
	label = -2;
      else
	label = class_map_.toId(tm.tracks_[i]->label_);
      
      for(size_t j = 0; j < tracks_[i]->frame_descriptors_.size(); ++j)
	tracks_[i]->frame_descriptors_[j]->label_ = label;
    }
  }

  TrackManagerCachedDescriptors::TrackManagerCachedDescriptors(const std::vector< boost::shared_ptr<TrackCachedDescriptors> >& tracks,
							       const NameMapping& class_map,
							       const NameMapping& descriptor_map) :
    tracks_(tracks),
    class_map_(class_map),
    descriptor_map_(descriptor_map)
  {
  }
  
  void TrackManagerCachedDescriptors::serialize(std::ostream& out) const
  {
    out << "TrackManagerCachedDescriptors" << endl;
    out << class_map_.serialize() << endl;
    out << descriptor_map_.serialize() << endl;
    size_t buf = tracks_.size();
    out.write((char*)&buf, sizeof(size_t));
    for(size_t i = 0; i < tracks_.size(); ++i)
      tracks_[i]->serialize(out);
  }

  void TrackManagerCachedDescriptors::save(const std::string& path) const
  {
    assert(path.substr(path.length() - 7).compare(".tm.mbd") == 0);
    ofstream file;
    file.open(path.c_str());
    serialize(file);
    file.close();
  }

  string TrackManagerCachedDescriptors::status() const
  {
    ostringstream oss;
    oss << class_map_.serialize() << endl;
    oss << descriptor_map_.serialize() << endl;
    oss << tracks_.size() << " tracks." << endl;
    return oss.str();
  }

  void TrackManagerCachedDescriptors::deserialize(std::istream& in)
  {
    assert(tracks_.empty());

    string line;
    getline(in, line);
    assert(line.compare("TrackManagerCachedDescriptors") == 0);

    class_map_ = NameMapping(in);
    descriptor_map_ = NameMapping(in);
    getline(in, line); // goddamn newline
    
    size_t num_tracks;
    in.read((char*)&num_tracks, sizeof(size_t));

    tracks_ = vector< shared_ptr<TrackCachedDescriptors> >(num_tracks);
    for(size_t i = 0; i < num_tracks; ++i)
      tracks_[i] = shared_ptr<TrackCachedDescriptors>(new TrackCachedDescriptors(this, in));
  }

  size_t TrackManagerCachedDescriptors::getTotalObjects() const
  {
    size_t total = 0;
    for(size_t i = 0; i < tracks_.size(); ++i)
      total += tracks_[i]->frame_descriptors_.size();

    return total;
  }


  TrackCachedDescriptors::TrackCachedDescriptors(TrackManagerCachedDescriptors* container,
						 std::vector<Object*> frame_descriptors,
						 Object* global_descriptors) :
    frame_descriptors_(frame_descriptors),
    global_descriptors_(global_descriptors),
    container_(container)
  {
  }
  
  TrackCachedDescriptors::TrackCachedDescriptors(TrackManagerCachedDescriptors* container,
						 istream& in) :
    global_descriptors_(NULL),
    container_(container)
  {
    deserialize(in);
  }

  TrackCachedDescriptors::~TrackCachedDescriptors()
  {
    for(size_t i = 0; i < frame_descriptors_.size(); ++i)
      delete frame_descriptors_[i];
  }

  void TrackCachedDescriptors::serialize(std::ostream& out)
  {
    out << "TrackCachedDescriptors" << endl;
    size_t buf = frame_descriptors_.size();
    out.write((char*)&buf, sizeof(size_t));
    for(size_t i = 0; i < frame_descriptors_.size(); ++i)
      frame_descriptors_[i]->serialize(out);
  }
  
  void TrackCachedDescriptors::deserialize(std::istream& in)
  {
    string line;
    getline(in, line);
    assert(line.compare("TrackCachedDescriptors") == 0);
    
    size_t num_frames;
    in.read((char*)&num_frames, sizeof(size_t));
    frame_descriptors_ = vector<Object*>(num_frames, NULL);
    
    for(size_t i = 0; i < frame_descriptors_.size(); ++i)
      frame_descriptors_[i] = new Object(in);
  }

  std::vector<Object*> TrackCachedDescriptors::copyObjects() const
  {
    vector<Object*> objs(frame_descriptors_.size(), NULL);
    for(size_t i = 0; i < frame_descriptors_.size(); ++i) {
      objs[i] = new Object(*frame_descriptors_[i]);
    }
    return objs;
  }

  int TrackCachedDescriptors::label() const
  {
    int label = frame_descriptors_[0]->label_;
    for(size_t i = 1; i < frame_descriptors_.size(); ++i)
      assert(label == frame_descriptors_[i]->label_);

    return label;
  }

  void TrackCachedDescriptors::setClassId(int class_id)
  {
    for(size_t i = 0; i < frame_descriptors_.size(); ++i) {
      frame_descriptors_[i]->label_ = class_id;
    }
  }

  std::string TrackCachedDescriptors::stringLabel() const
  {
    int int_label = label();
    string str;
    if(int_label == -1)
      str = "background";
    else if(int_label == -2)
      str = "unlabeled";
    else
      str = container_->class_map_.toName(int_label);

    return str;
  }
  
  uint64_t TrackCachedDescriptors::numBytes() const
  {
    uint64_t num_bytes = 0;
    for(size_t i = 0; i < frame_descriptors_.size(); ++i) {
      num_bytes += frame_descriptors_[i]->numBytes();
      if(global_descriptors_)
	num_bytes += global_descriptors_->numBytes();
    }
    return num_bytes;
  }

  uint64_t TrackManagerCachedDescriptors::numBytes() const
  {
    uint64_t num_bytes = 0;
    for(size_t i = 0; i < tracks_.size(); ++i)
      num_bytes += tracks_[i]->numBytes();
    return num_bytes;
  }
  
  void TrackManagerCachedDescriptors::addTrack(boost::shared_ptr<TrackCachedDescriptors> track)
  {
    assert(track->container_);
    assert(track->container_->class_map_.compare(class_map_)); // TODO: Add support for remapping.
    track->container_ = this;
    tracks_.push_back(track);
  }
  
  vector<Object*> TrackManagerCachedDescriptors::copyObjects() const
  {
    vector<Object*> objs(getTotalObjects(), NULL);
    size_t idx = 0;
    for(size_t i = 0; i < tracks_.size(); ++i)
      for(size_t j = 0; j < tracks_[i]->frame_descriptors_.size(); ++j, ++idx)
	objs[idx] = new Object(*tracks_[i]->frame_descriptors_[j]);

    return objs;
  }
  
} // namespace
