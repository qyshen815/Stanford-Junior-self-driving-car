/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#include "aw_Graph.h"

using namespace std;

namespace vlr {

namespace RoutePlanner {

/*
  Edge::Edge(Vertex * from, Vertex * to, double weight, int id)
    : visited( false ), from(from), to(to), id(id), weight(weight), annotatedEdge(0)
  {
  }

  Vertex * Graph::addVertex() {
    return addVertex(idCount.getNextId());
  }

  Vertex * Graph::addVertex(int id) {
    Vertex * v = new Vertex(id);
    vertexMap.insert(make_pair(id, v));
    return v;
  }

  Edge * Graph::connect(Vertex * v1, Vertex * v2, double weight) {
    return connect(v1, v2, weight, idCount.getNextId());
  }

  Edge * Graph::connect(Vertex * v1, Vertex * v2, double weight, int edgeId) {
    Edge * edge = new Edge(v1, v2, weight, edgeId);
    v1->addEdge(edge);
    edgeMap.insert(make_pair(edgeId, edge));
    return edge;
  }

  Vertex * Graph::getVertex(int id) {
    return vertexMap.find(id)->second;
  }

  Edge * Graph::getEdge(int id) {
    return edgeMap.find(id)->second;
  }

  Graph::~Graph() {
    for (VertexMap::iterator vit = vertexMap.begin(); vit != vertexMap.end(); ++vit) {
      delete vit->second;
    }
    for (EdgeMap::iterator eit = edgeMap.begin(); eit != edgeMap.end(); ++eit) {
      delete eit->second;
    }
  }
  */

}

} // namespace vlr
