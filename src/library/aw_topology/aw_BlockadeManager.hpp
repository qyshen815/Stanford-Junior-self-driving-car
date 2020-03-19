/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_BLOCKADE_MANAGER_HPP
#define AW_BLOCKADE_MANAGER_HPP

#include <aw_RndfGraph.h>

namespace vlr {

using namespace RoutePlanner;

class Topology;

/*this struct servers to return info about road blockades.*/
struct BlockadeInfo{
      float thres_critical_zdif; //specify that value, you will get number of cells that violoate (are above) this threshold
      double borderDist;      //specify that value, the width to each side of the track spanning the area to verify
      int thres_cirticalN;    //specify that value, if nAboveThreshold is above this threshold blocked will be set to true
      int nAboveThreshold;    //you receive that value, the number of cells that violoate @threshold_to_use
      float maxValue;         //you recieve that value, the maximum value in the area
      float averageValue;     //the average value in the area


   /*says whether the path blocked*/
   bool bBlocked;
   /*says whether verification was successfull, only in this case @blocked can be trusted*/
   bool bVerificationWasSuccessfull;
};

//--------------------------------------------------------
//             BlockadeManager
//--------------------------------------------------------

class BlockadeManager {
public:
  class Blockade {
  public:
    Blockade();
    Blockade(RndfEdge* edge);
    ~Blockade();
    void update();
    void publish();
    void forcePublish();
    double getLastUpdateTime();

    RndfEdge* getEdge() const { return edge; }
    bool isPublished() const { return published; }
    double getLastUpdateTime() const { return last_update_time; }

  private:
    RndfEdge* edge;
    int update_counter;
    bool published;
    double last_update_time;
  };
  typedef std::map<RndfEdge*, Blockade> TBlockageMap;

  BlockadeManager(Topology* topology);

  bool update();
  TBlockageMap& getBlockades(void) { return blockade_map; }

  Blockade* getBlockadeByEdge(RndfEdge* edge);
  void addBlockade(RndfEdge* edge);
  void forceBlockade(RndfEdge* edge);
  void deleteBlockade(RndfEdge* edge);

public:
  TBlockageMap blockade_map; // this map holds all blockages
  Topology* topology;
  int current_blockade_id;
};


inline std::ostream& operator << (std::ostream& os, const BlockadeManager::Blockade& v)
{
	return os << "Blockade: "<< v.getEdge() <<"  last update: "<< v.getLastUpdateTime() <<"  "<< (v.isPublished() ? "PUBLISHED" : "NOT PUBLISHED");
}

} // namespace vlr

#endif
