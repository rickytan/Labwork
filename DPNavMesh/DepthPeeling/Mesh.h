#pragma once

#pragma warning(disable:4996)
#include <vector>
#include <limits>

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <iostream>

#ifdef min
#undef min
#endif

// stuff to define the mesh
#include <vcg/complex/complex.h>
#include <vcg/simplex/face/base.h>
#include <vcg/simplex/edge/base.h>
#include <vcg/complex/complex.h>

// io
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export.h>

// update
#include <vcg/complex/algorithms/update/topology.h>

// gl
#include <gl/glew.h>
#include <wrap/gl/trimesh.h>



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
public:
    Mesh(): vcg::tri::TriMesh<std::vector<Vertex>, std::vector<Face>>() {}
};