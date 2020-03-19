#ifndef TRACK_MANAGER_CACHED_DESCRIPTORS_H
#define TRACK_MANAGER_CACHED_DESCRIPTORS_H

#include <track_manager.h>
#include <multibooster/multibooster.h>
#include <multibooster_support.h>

namespace track_manager { 

  class TrackManagerCachedDescriptors;
  
  class TrackCachedDescriptors
  {
  public:
    typedef boost::shared_ptr<TrackCachedDescriptors> Ptr;
    
    //! Freed at destruction.
    std::vector<Object*> frame_descriptors_;
    Object* global_descriptors_;
    //! Back pointer to the containing TrackManagerCachedDescriptors object.  It has the name mappings.
    TrackManagerCachedDescriptors* container_;
    
    TrackCachedDescriptors(TrackManagerCachedDescriptors* container, std::vector<Object*> frame_descriptors, Object* global_descriptors = NULL);
    TrackCachedDescriptors(TrackManagerCachedDescriptors* container, std::istream& in);
    ~TrackCachedDescriptors();
    void serialize(std::ostream& out);
    std::vector<Object*> copyObjects() const;
    int label() const;
    void setClassId(int class_id);
    std::string stringLabel() const;
    uint64_t numBytes() const;

  private:
    void deserialize(std::istream& in);
  };
  

  class TrackManagerCachedDescriptors
  {
  public:
    typedef boost::shared_ptr<TrackManagerCachedDescriptors> Ptr;
    
    //! Don't add to this directly.
    std::vector< boost::shared_ptr<TrackCachedDescriptors> > tracks_;
    NameMapping class_map_;
    NameMapping descriptor_map_;
    
    //! Loads descriptors from path.
    TrackManagerCachedDescriptors(const std::string& path);
    //! Computes all descriptors for the given track_manager.
    TrackManagerCachedDescriptors(const TrackManager& tm, const NameMapping& class_map);
    TrackManagerCachedDescriptors(const std::vector< boost::shared_ptr<TrackCachedDescriptors> >& tracks,
				  const NameMapping& class_map,
				  const NameMapping& descriptor_map);
    void save(const std::string& path) const;
    void serialize(std::ostream& out) const;
    std::string status() const;
    size_t getTotalObjects() const;
    std::vector<Object*> copyObjects() const;
    //! Adds to tracks_, sets its container_ properly.
    void addTrack(boost::shared_ptr<TrackCachedDescriptors> track);
    uint64_t numBytes() const;

  private:
    void deserialize(std::istream& in);
  };
  
}

#endif // TRACK_MANAGER_CACHED_DESCRIPTORS_H
