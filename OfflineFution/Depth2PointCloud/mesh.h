
#ifndef __MESH_H_
#define __MESH_H_

#pragma warning(disable:4996)
#pragma warning(disable:4244) // double to float


#include <vector>


// stuff to define the mesh
#include <vcg/complex/complex.h>
#include <vcg/simplex/face/base.h>
#include <vcg/simplex/edge/base.h>

#include <vcg/math/quadric.h>
#include <vcg/complex/algorithms/clean.h>

// io
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export.h>

// update
#include <vcg/complex/algorithms/update/topology.h>

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
	> () {}

};


class Edge: public vcg::Edge<UsedTypes> {};

class Face: public vcg::Face<
    UsedTypes,
    vcg::face::VFAdj,
    vcg::face::VertexRef,
    vcg::face::BitFlags
> {};

class Mesh: public vcg::tri::TriMesh<std::vector<Vertex>, std::vector<Face>> {
};

#undef static_assert
#endif