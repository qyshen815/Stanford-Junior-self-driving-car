#ifndef AW_CHECKPOINTLIST_H
#define AW_CHECKPOINTLIST_H

#include <iostream>
#include <deque>

#include <aw_checkPoint.h>

namespace vlr {

class Topology;

namespace rndf {

  typedef std::deque<CheckPoint*> TCheckpointVec;

  class CheckpointList : public NetElement {

  public:
    CheckpointList(uint32_t id, std::string & strName);
    virtual ~CheckpointList();

    inline size_t size() {return vector_.size();}
    inline bool empty() {return vector_.empty();}

    virtual bool addCheckpoint(CheckPoint* pCheckpoint, bool atBeginning=false);

    TCheckpointVec::const_iterator begin() const { return vector_.begin(); }
    TCheckpointVec::const_iterator end() const { return vector_.end(); }

    TCheckpointVec& getCheckpoints() { return vector_; }

    void dump();

  private:
    TCheckpointVec vector_;

    friend class Topology;
  };

}

} // namespace vlr

#endif
