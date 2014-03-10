
#ifndef __BI_TRI_EDGE_COLLAPSE_QUARDIC_H
#define __BI_TRI_EDGE_COLLAPSE_QUARDIC_H


#include <vcg/complex/algorithms/local_optimization.h>
#include <vcg/complex/algorithms/local_optimization/tri_edge_collapse_quadric.h>

namespace vcg {
	namespace tri {

		template <class CoordType>
		class BiTriEdgeCollapseQuadricParameter: public TriEdgeCollapseQuadricParameter
		{
		public:
			BiTriEdgeCollapseQuadricParameter(): alignDir(CoordType(0, 0, 1)), TriEdgeCollapseQuadricParameter() {}
			CoordType alignDir;
		};

		template<
			class TriMeshType, 
			class MYTYPE, 
			class VertexPairType = vcg::tri::BasicVertexPair<typename TriMeshType::VertexType>
		> 
		class BiTriEdgeCollapse: public vcg::tri::TriEdgeCollapseQuadric<
			TriMeshType,
			VertexPairType,
			MYTYPE
		> {
		public:
			typedef VertexPairType VertexPair;
			typedef BiTriEdgeCollapseQuadricParameter<typename TriMeshType::CoordType> QParameter;

			BiTriEdgeCollapse(const VertexPairType &p, int i, vcg::BaseParameterClass *pp): vcg::tri::TriEdgeCollapseQuadric<
				TriMeshType,
				VertexPairType,
				MYTYPE
			> (p, i, pp) {}

			void Execute(TriMeshType &m0, TriMeshType &m1, vcg::BaseParameterClass *_pp) {

			}

			void Execute(TriMeshType &m, vcg::BaseParameterClass *_pp) {
				QParameter *pp=(QParameter *)_pp;
				CoordType newPos;
				if(pp->OptimalPlacement) 
					newPos= this->ComputeMinimal();
				else
					newPos=this->pos.V(1)->P();

				QH::Qd(this->pos.V(1))+=QH::Qd(this->pos.V(0));
				EdgeCollapser<TriMeshType,VertexPairType>::Do(m, this->pos, newPos); // v0 is deleted and v1 take the new position
			}
		};

	}
}

#endif