/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#include "aw_LaneChangeManager.hpp"

using namespace std;

namespace vlr {

#define TRACE(str) cout << "[LaneChangeManager] " << str << endl;

LaneChangeManager::LaneChangeManager(Topology* top, VehicleManager* vman)
: top(top), graph(NULL), vman(vman)
{
	assert(top);
	graph = top->complete_graph;
	assert(graph);
	assert(vman);
}

LaneChangeManager::~LaneChangeManager()
{
}

void LaneChangeManager::annotateLaneChanges(Topology* /*top*/) {

}

} // namespace vlr
