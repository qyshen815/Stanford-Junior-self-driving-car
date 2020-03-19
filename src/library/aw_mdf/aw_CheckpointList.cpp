#include "aw_CheckpointList.h"

namespace vlr {

namespace rndf {

CheckpointList::CheckpointList(uint32_t id, std::string& strName) : NetElement(id, strName) {
}

CheckpointList::~CheckpointList() {
}

bool CheckpointList::addCheckpoint(CheckPoint* pCheckpoint, bool atBeginning) {
  if (atBeginning) {
    vector_.insert(vector_.begin(), pCheckpoint);
  }
  else {
    vector_.push_back(pCheckpoint);
  }
  return true;
}


void CheckpointList::dump() {
  for (TCheckpointVec::iterator it = vector_.begin(); it != vector_.end(); ++it) {
    (*it)->dump();
  }
}
}

} // namespace vlr
