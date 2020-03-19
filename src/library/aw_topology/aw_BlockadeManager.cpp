/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#include <iostream>
#include <global.h>
#include "aw_BlockadeManager.hpp"

namespace vlr {

using namespace RoutePlanner;

#undef TRACE
#define TRACE(str)
//#define TRACE(str) std::cout << "[BlockadeManager] " << str << std::endl;

#define BLOCKADE_PUBLISH_MIN          30
#define BLOCKADE_MAX_TIME_UNOBSERVED  60.0
//-----------------------------------------------------------------------------
//             Blockade
//-----------------------------------------------------------------------------
BlockadeManager::Blockade::Blockade() : edge(NULL), update_counter(0), published(false)
{
}

BlockadeManager::Blockade::Blockade(RndfEdge* edge) : edge(edge), update_counter(0), published(false)
{
}

BlockadeManager::Blockade::~Blockade()
{
}

void BlockadeManager::Blockade::update()
{
  last_update_time = Time::current();
  update_counter++;
  if(update_counter > BLOCKADE_PUBLISH_MIN) {
    publish();
  }
}

void BlockadeManager::Blockade::publish()
{
  edge->setBlocked();
  published = true;
}

double BlockadeManager::Blockade::getLastUpdateTime()
{
  return last_update_time;
}

void BlockadeManager::Blockade::forcePublish()
{
  last_update_time = Time::current();
  update_counter = BLOCKADE_PUBLISH_MIN+1;
  publish();
}
//-----------------------------------------------------------------------------
//             BlockadeManager
//-----------------------------------------------------------------------------

BlockadeManager::BlockadeManager(Topology* topology) :
	topology(topology), current_blockade_id(0)
{
}

BlockadeManager::Blockade* BlockadeManager::getBlockadeByEdge(RndfEdge* edge) {
  TBlockageMap::iterator it;
  if(edge==NULL) return NULL;

  it = blockade_map.find(edge);
  if(it == blockade_map.end()) {
    return NULL;
  }
  else {
    Blockade& blockade = it->second;
    return &blockade;
  }
}

void BlockadeManager::addBlockade(RndfEdge* edge)
{
  TBlockageMap::iterator it;
  assert(edge);

  // Zone Edges auslassen
  if (edge->isBlockedEdge()) return;
  if (edge->isZoneEdge()) return;
  if (edge->getIntersection()) return;

  it = blockade_map.find(edge);
  if(it == blockade_map.end()) {
    blockade_map[ edge ] = Blockade(edge);
    blockade_map[ edge ].update();
  }
  else {
    Blockade& blockade = it->second;
    blockade.update();
  }
}

void BlockadeManager::forceBlockade(RndfEdge* edge)
{
  TBlockageMap::iterator it;
  assert(edge);

  // Zone Edges auslassen
  if (edge->isBlockedEdge()) return;
  if (edge->isZoneEdge()) return;
  //if (edge->getIntersection()) return; // forceBlockade gets called to block intersections

  it = blockade_map.find(edge);
  if(it == blockade_map.end()) {
    blockade_map[ edge ] = Blockade(edge);
    blockade_map[ edge ].forcePublish();
  }
  else {
    Blockade& blockade = it->second;
    blockade.forcePublish();
  }
}

void BlockadeManager::deleteBlockade(RndfEdge* edge)
{
  blockade_map.erase(edge);
}

bool BlockadeManager::update( )
{
  TBlockageMap::iterator it, it_end;
  double current_timestamp = Time::current();
  double diff_time;
  std::vector< RndfEdge* > deleted_objects;

//  TRACE("Aktuelle Blockaden:");
  for(it = blockade_map.begin(), it_end = blockade_map.end(); it!=it_end; ++it) {
	  Blockade& blockade = it->second;
//	  TRACE("  "<< blockade);
    diff_time = current_timestamp-blockade.getLastUpdateTime();
    // if blockade was not observed for some time -> erase from blockade manager
    if(diff_time>BLOCKADE_MAX_TIME_UNOBSERVED) {
      deleted_objects.push_back(it->first);
    } else if (blockade.isPublished() && blockade.getEdge()) {
    	blockade.getEdge()->setBlocked();
    }
  }

  std::vector< RndfEdge* >::iterator itd, itd_end;
  for(itd = deleted_objects.begin(), itd_end = deleted_objects.end(); itd!=itd_end; ++itd) {
    blockade_map.erase(*itd);
  }

  return true;
}

} // namespace vlr
