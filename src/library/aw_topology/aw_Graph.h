/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_GRAPH_H
#define AW_GRAPH_H

#include <list>
#include <map>
#include <set>
#include <assert.h>

#include "aw_Counter.h"
//#include "aw_Route.h"

using std::set;

namespace vlr {

namespace RoutePlanner {

class AnnotatedRouteEdge;

template<class EdgeT> class Vertex {
public:
	typedef std::list<EdgeT *>					EdgeList;
	typedef typename EdgeList::const_iterator	EdgeIterator;
	typedef set<EdgeT*>							TEdgeSet;

	Vertex(int id = -1) : id(id) {}
	virtual ~Vertex() {}

	// deprecated interface
	void addEdge(EdgeT * edge) { assert(edge); assert(edge > (EdgeT *)0x040000); edgeList.push_back(edge); }
	void removeEdge(EdgeT * edge) { edgeList.remove(edge); }
	typename EdgeList::iterator beginEdges() { return edgeList.begin(); }
	typename EdgeList::iterator endEdges() { return edgeList.end();	}
	EdgeIterator beginEdges() const { return edgeList.begin(); }
	EdgeIterator endEdges() const {	return edgeList.end(); }
	int getNumberOfEdges() const { return edgeList.size(); }

	// new interface
	void addInEdge(EdgeT* edge)					{ assert(edge); assert(edge > (EdgeT *)0x040000); m_inEdges.insert(edge); }
	void removeInEdge(EdgeT* edge)				{ assert(edge); m_inEdges.erase(edge); }
	bool hasInEdge(EdgeT* edge) const		{ return m_inEdges.find(edge) != m_inEdges.end(); }
	size_t numInEdges() const				{ return m_inEdges.size(); }
	const TEdgeSet& getInEdges() const			{ return m_inEdges; }

	void addOutEdge(EdgeT* edge)				{ assert(edge); assert(edge > (EdgeT *)0x040000); m_outEdges.insert(edge); }
	void removeOutEdge(EdgeT* edge)				{ assert(edge); m_outEdges.erase(edge); }
	bool hasOutEdge(EdgeT* edge) const			{ return m_outEdges.find(edge) != m_outEdges.end(); }
	size_t numOutEdges() const			{ return m_outEdges.size(); }
	const TEdgeSet& getOutEdges() const			{ return m_outEdges; }


	int getId() const {	return id; }

protected:
	EdgeList edgeList;
	TEdgeSet m_outEdges;
	TEdgeSet m_inEdges;

	int id;
};

template<class VertexT> class Edge {
public:
	Edge(VertexT * from, VertexT * to, double weight = 1., int id = -1)
	: visited( false ), from(from), to(to), id(id), weight(weight)/*, annotatedEdge(0)*/
	{
		assert(weight > 0.); // has to be greater than 0. otherwise the AStar Route Planner fails
	}
	virtual ~Edge() { }

	void deleteVertices() {
		delete to;
		to = 0;

		delete from;
		from = 0;
	}

	virtual VertexT * fromVertex() {
		return from;
	}
	virtual VertexT * toVertex() {
		return to;
	}
	virtual VertexT const * fromVertex() const {
		return from;
	}
	virtual VertexT const * toVertex() const {
		return to;
	}
	int getId() const {
		return id;
	}
	double getWeight() const {
		return weight;
	}

	/* this is evil :) RndfEdge <-> AnnotetedRouteEdge is no 1:1 relation
	// (sk) get link to annotaion if available (if edge is in route graph)
	AnnotatedRouteEdge* getAnnotatedEdge() {
		return annotatedEdge;
	}
	const AnnotatedRouteEdge* getAnnotatedEdge() const {
		return annotatedEdge;
	}
	void setAnnotatedEdge(AnnotatedRouteEdge* annoEdge) {
		annotatedEdge=annoEdge;
	}
	*/


	// (jz) helps with some algorithms
	mutable bool visited;

protected:
	VertexT * from;
	VertexT * to;
	int id;
	double weight;
//	AnnotatedRouteEdge* annotatedEdge;
};

template<class VertexT, class EdgeT> class Graph {
public:
	typedef std::map<int, VertexT*> VertexMap;
	typedef std::map<int, EdgeT*> EdgeMap;
	typedef std::list<EdgeT*> EdgeList;
	typedef std::set<EdgeT*> EdgeSet;

	Graph() {
	}
	virtual ~Graph();

	VertexT * addVertex();
	VertexT * addVertex(int id);
	EdgeT * connect(VertexT * v1, VertexT * v2, double weight = 1.);
	EdgeT * connect(VertexT * v1, VertexT * v2, double weight, int edgeId);
	void removeEdge(EdgeT * edge);

	VertexT * getVertex(int id);
	EdgeT * getEdge(int id);

	EdgeT * findIncomingEdge(VertexT * to);
	int countIncomingEdge(VertexT * to);

	EdgeList * searchPath(VertexT * from, VertexT * to, bool ignore_blockades = false);

	// returns a set of edges that are according to the graph
	EdgeSet getEdges();

protected:
	virtual double searchHeuristic(const VertexT* /*vertex*/, const VertexT* /*goal*/) {
		return 0.;
	}
	// (jz) made this public
public:
	EdgeMap edgeMap;
protected:
	VertexMap vertexMap;
	rndf::Counter idCount;
};

template<class VertexT, class EdgeT> VertexT * Graph<VertexT, EdgeT>::addVertex() {
	return addVertex(idCount.getNextId());
}

template<class VertexT, class EdgeT> VertexT * Graph<VertexT, EdgeT>::addVertex(int id) {
	VertexT * v = new VertexT(id);
	vertexMap.insert(make_pair(id, v));
	return v;
}

template<class VertexT, class EdgeT> EdgeT * Graph<VertexT, EdgeT>::connect(
		VertexT * v1, VertexT * v2, double weight) {
	return connect(v1, v2, weight, idCount.getNextId());
}

template<class VertexT, class EdgeT> EdgeT * Graph<VertexT, EdgeT>::connect(
		VertexT * v1, VertexT * v2, double weight, int edgeId)
{
	assert(v1 && v2);
	EdgeT * edge = new EdgeT(v1, v2, weight, edgeId);
	v1->addEdge(edge);
	v1->addOutEdge(edge);
	v2->addInEdge(edge);
	edgeMap.insert(make_pair(edgeId, edge));
	return edge;
}

template<class VertexT, class EdgeT> VertexT * Graph<VertexT, EdgeT>::getVertex(int id) {
	typename VertexMap::iterator it = vertexMap.find(id);
	if (it == vertexMap.end()) {
		return NULL;
	} else {
		return it->second;
	}
}

template<class VertexT, class EdgeT> EdgeT * Graph<VertexT, EdgeT>::getEdge(int id) {
	typename EdgeMap::iterator it = edgeMap.find(id);
	if (it == edgeMap.end()) {
		return NULL;
	} else {
		return it->second;
	}
}

template<class VertexT, class EdgeT> EdgeT * Graph<VertexT, EdgeT>::findIncomingEdge(VertexT * to) {
	// #warning this looks sick. consider adapting vertex and edge types to allow backward traversal (jz)
	for (typename EdgeMap::iterator it = edgeMap.begin(); it != edgeMap.end(); ++it) {
		if (it->second->toVertex() == to) {
			return it->second;
		}
	}
	return NULL;
}

template<class VertexT, class EdgeT> int Graph<VertexT, EdgeT>::countIncomingEdge(VertexT * to) {
	int count=0;
	for (typename EdgeMap::iterator it = edgeMap.begin(); it != edgeMap.end(); ++it) {
		if (it->second->toVertex() == to) {
			++count;
		}
	}
	return count;
}

template<class VertexT, class EdgeT> Graph<VertexT, EdgeT>::~Graph() {
	for (typename VertexMap::iterator vit = vertexMap.begin(); vit != vertexMap.end(); ++vit) {
		delete vit->second;
	}
	for (typename EdgeMap::iterator eit = edgeMap.begin(); eit != edgeMap.end(); ++eit) {
		delete eit->second;
	}
}

template<class VertexT, class EdgeT> void Graph<VertexT, EdgeT>::removeEdge(EdgeT * edge) {
	assert(edge->fromVertex());
	assert(edge->toVertex());
	edge->fromVertex()->removeEdge(edge);
	edge->fromVertex()->removeOutEdge(edge);
	edge->toVertex()->removeInEdge(edge);
	//typename EdgeMap::iterator it = find(edgeMap.begin(), edgeMap.end(), edge->getId());
	typename EdgeMap::iterator it = edgeMap.find(edge->getId());
	assert(it != edgeMap.end());
	edgeMap.erase(it);
	delete edge;
}

template<class VertexT, class EdgeT> typename Graph<VertexT, EdgeT>::EdgeSet Graph<VertexT, EdgeT>::getEdges() {
	EdgeSet edges;
	for (typename EdgeMap::iterator it = edgeMap.begin(); it != edgeMap.end(); ++it)
		edges.insert(it->second);
	return edges;
}

}

} // namespace vlr

#endif
