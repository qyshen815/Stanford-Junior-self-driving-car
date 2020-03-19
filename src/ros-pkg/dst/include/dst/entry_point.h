#ifndef ENTRY_POINT_H
#define ENTRY_POINT_H

#include <pipeline2/pipeline2.h>
#include <dst/typedefs.h>

namespace dst
{
  
  //! Class to manage input of data to a pipeline2.
  //! T will be copied a bit so it should generally be something small,
  //! like a pointer, shared_ptr, or POD type.
  template<typename T>
  class EntryPoint : public pipeline2::ComputeNode
  {
  public:
    std::string tag_;
    pipeline2::Outlet<T> outlet_;
  
    EntryPoint(const std::string& tag = "");
    void setData(T);
    ~EntryPoint() {};
    
  protected:
    void _compute() {};
    void _display() const {};
    void _flush() {};
    std::string _getName() const;
  };
  
  
  /************************************************************
   * Template implementation.
   ************************************************************/
  
  template<typename T>
  EntryPoint<T>::EntryPoint(const std::string& tag) :
    tag_(tag),
    outlet_(this)
  {
  }
  
  template<typename T>
  std::string EntryPoint<T>::_getName() const
  {
    std::ostringstream oss;
    oss << "EntryPoint-" << tag_;
    return oss.str();
  }

  template<typename T>
  void EntryPoint<T>::setData(T data)
  {
    outlet_.push(data);
  }

  /************************************************************
   * Helper functions & template implementations.
   ************************************************************/
  
  template <typename T>
  EntryPoint<T>*
  getEntryPoint(const std::vector<pipeline2::ComputeNode*>& nodes)
  {
    std::vector<EntryPoint<T>*> matches = pipeline2::filterNodes< EntryPoint<T> >(nodes);
    if(matches.empty())
      return NULL;
    else if(matches.size() == 1)
      return matches[0];
    else {
      ROS_FATAL("getEntryPoint could not find a unique entry point.");
      return NULL;
    }
  }
  
  template <typename T>
  EntryPoint<T>*
  getTaggedEntryPoint(const std::vector<pipeline2::ComputeNode*>& nodes,
		      const std::string& tag)
  {
    std::vector<EntryPoint<T>*> x = pipeline2::filterNodes< EntryPoint<T> >(nodes);
    std::vector<EntryPoint<T>*> matches;
    matches.reserve(x.size());
    
    for(size_t i = 0; i < x.size(); ++i) {
      if(x[i]->tag_.compare(tag) == 0)
	matches.push_back(x[i]);
    }
         
    if(matches.empty())
      return NULL;
    else if(matches.size() == 1)
      return matches[0];
    else {
      ROS_FATAL("Two entry points cannot share the same tag.");
      return NULL;
    }
  }

  
} // namespace dst

#endif // ENTRY_POINT_H
