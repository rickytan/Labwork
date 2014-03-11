
#ifndef __MESH_H_
#define __MESH_H_

#pragma warning(disable:4996)
#pragma warning(disable:4244) // double to float


#include <vector>
#include <limits>

#include <stdio.h>
#include <stdlib.h>

// stuff to define the mesh
#include <vcg/complex/complex.h>
#include <vcg/simplex/face/base.h>
#include <vcg/simplex/edge/base.h>
#include <vcg/complex/complex.h>

#include <vcg/math/quadric.h>
#include <vcg/complex/algorithms/clean.h>

// io
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export.h>

// update
#include <vcg/complex/algorithms/update/topology.h>

// local optimization
#include <vcg/complex/algorithms/local_optimization.h>
#include <vcg/complex/algorithms/local_optimization/tri_edge_collapse_quadric.h>

// My Implement
#include "bi_tri_edge_collapse_quadric.h"
#include "bi_optimization.h"


class Vertex;
class Edge;
class Face;

class UsedTypes: public vcg::UsedTypes<
	vcg::Use<Vertex>::AsVertexType,
	vcg::Use<Edge>::AsEdgeType,
	vcg::Use<Face>::AsFaceType
>{};

class Vertex: public vcg::Vertex<
	UsedTypes,
	vcg::vertex::VFAdj,
	vcg::vertex::Coord3f,
	vcg::vertex::Normal3f,
	vcg::vertex::Mark,
	vcg::vertex::BitFlags
> {
public:
	Vertex(): vcg::Vertex<
		UsedTypes,
		vcg::vertex::VFAdj,
		vcg::vertex::Coord3f,
		vcg::vertex::Normal3f,
		vcg::vertex::Mark,
		vcg::vertex::BitFlags
	> (), _cv(NULL) {}
	vcg::math::Quadric<double> &Qd() {return q;}
	Vertex *&Cv() {return _cv;}

private:
	vcg::math::Quadric<double> q;
	// the Corresponding vertex on the other mesh
	Vertex *_cv;
};

class Edge: public vcg::Edge<UsedTypes> {};

class Face: public vcg::Face<
	UsedTypes,
	vcg::face::VFAdj,
	vcg::face::VertexRef,
	vcg::face::BitFlags
> {};

typedef vcg::tri::BasicVertexPair<Vertex> VertexPair;

class Mesh: public vcg::tri::TriMesh<std::vector<Vertex>, std::vector<Face>> {};

class TriEdgeCollapse: public vcg::tri::TriEdgeCollapseQuadric<
	Mesh,
	VertexPair,
	TriEdgeCollapse,
	vcg::tri::QInfoStandard<Vertex>
> {
public:
	TriEdgeCollapse(const VertexPair &p, int i, vcg::BaseParameterClass *pp): vcg::tri::TriEdgeCollapseQuadric<
		Mesh,
		VertexPair,
		TriEdgeCollapse,
		vcg::tri::QInfoStandard<Vertex>
		> (p, i, pp) {}

};

class MyTriEdgeCollapse: public vcg::tri::BiTriEdgeCollapse<
	Mesh, 
	MyTriEdgeCollapse
> {
public:
	MyTriEdgeCollapse(const VertexPair &p, int i, vcg::BaseParameterClass *pp): vcg::tri::BiTriEdgeCollapse<
		Mesh,
		MyTriEdgeCollapse
	> (p, i, pp) {}
};

#endif