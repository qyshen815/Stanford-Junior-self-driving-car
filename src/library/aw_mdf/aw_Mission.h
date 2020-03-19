#ifndef AW_MISSION_H
#define AW_MISSION_H

#include "aw_roadNetwork.h"
#include "aw_CheckpointList.h"
#include "aw_SpeedLimitList.h"

namespace vlr {

namespace rndf {

  class Mission : public NetElement {
  public:
    Mission(RoadNetwork* rndf);
    virtual ~Mission();
    void clear();

    bool loadMDF(const std::string& strFileName);

    CheckpointList * addCheckpointList(const std::string & strData);
    SpeedLimitList * addSpeedLimitList(const std::string & strData);
    SpeedLimit * addSpeedLimit(const std::string & strName, const std::string & strMin, const std::string & strMax);

    inline void setName(std::string strName) {name_ = strName;}
    inline const std::string& name() {return name_;}
    void setStatus(std::string strStatus) {status_ = strStatus;}
    void appendStatus(std::string strStatus) {status_ += strStatus + '\n';}
    const std::string& status() {return status_;}

    CheckpointList* getCheckpointList() {return m_checkpointList;}

    void dump();

  private:
    CheckpointList* m_checkpointList;
    SpeedLimitList* m_speedLimitList;

    std::string status_;
    std::string name_;
    float format_version_;
    std::string creation_date_;
    RoadNetwork* m_rndf;
    std::string m_rndfName;

    void changeSpeedLimit(SpeedLimit* limit, const std::string & strMin, const std::string & strMax);
  };

}

} // namespace vlr

#endif
