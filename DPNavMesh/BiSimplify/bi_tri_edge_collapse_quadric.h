
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

		template <class MeshType>
		class BiVertexPair: public BasicVertexPair<typename MeshType::VertexType>
		{
			typedef typename MeshType::VertexType VertexType;
		public:
			inline BiVertexPair(): BasicVertexPair<VertexType>(), m(NULL) {}
			//inline BiVertexPair( VertexType *v0, VertexType *v1): BasicVertexPair<VertexType>(v0, v1), m(NULL) {}
			inline BiVertexPair( VertexType *v0, VertexType *v1, MeshType &mesh): BasicVertexPair<VertexType>(v0, v1), m(&mesh) {}
			inline BiVertexPair( VertexType *v0, VertexType *v1, MeshType *mesh): BasicVertexPair<VertexType>(v0, v1), m(mesh) {}
			MeshType *m;
		};

		template<
			class TriMeshType, 
			class MYTYPE, 
			class VertexPairType = BiVertexPair<TriMeshType>
		> 
		class BiTriEdgeCollapse: public vcg::tri::TriEdgeCollapseQuadric<
			TriMeshType,
			VertexPairType,
			MYTYPE
		> {
		public:
			typedef VertexPairType VertexPair;
			typedef BiTriEdgeCollapseQuadricParameter<typename TriMeshType::CoordType> QParameter;

			BiTriEdgeCollapse(const VertexPairType &p, int i, vcg::BaseParameterClass *pp)
			{
				this->localMark = i;
				this->pos=p;
				ScalarType _pri0 = ComputePriority(pp);
				this->pos = VertexPairType(p.cV(0)->Cv(), p.cV(1)->Cv(), p.m->Cm());
				ScalarType _pri1 = ComputePriority(pp);
				if (_pri0 < _pri1) {
					this->_priority = _pri1;
				}
				else {
					this->_priority = _pri0;
					this->pos = p;
				}
			}

			void Execute(TriMeshType &m, vcg::BaseParameterClass *_pp) {
				QParameter *pp=(QParameter *)_pp;
				CoordType newPos0, newPos1;
				VertexPairType vp0, vp1;
				vp0 = this->pos;
				vp1 = VertexPairType(vp0.V(0)->Cv(), vp0.V(1)->Cv(), vp0.m->Cm());
				if(pp->OptimalPlacement) {
					newPos0= this->ComputeMinimal();
					this->pos = vp1;
					newPos1 = this->ComputeMinimal();
				}
				else {
					newPos0=this->pos.V(1)->P();
					newPos1=this->pos.V(1)->Cv()->P();
				}
				newPos1[0] = newPos0[0];
				newPos1[1] = newPos0[1];

				QH::Qd(vp0.V(1))+=QH::Qd(vp0.V(0));
				EdgeCollapser<TriMeshType,VertexPairType>::Do(*vp0.m, vp0, newPos0); // v0 is deleted and v1 take the new position
				QH::Qd(vp1.V(1))+=QH::Qd(vp1.V(0));
				EdgeCollapser<TriMeshType,VertexPairType>::Do(*vp1.m, vp1, newPos1); // v0 is deleted and v1 take the new position
				this->pos = vp0;
			}

			inline  void UpdateHeap(HeapType & h_ret,BaseParameterClass *_pp)
			{
				QParameter *pp=(QParameter *)_pp;
				this->GlobalMark()++;
				VertexType *v[2];
				v[0]= this->pos.V(0);
				v[1]= this->pos.V(1);
				v[1]->IMark() = this->GlobalMark();

				// First loop around the remaining vertex to unmark visited flags
				vcg::face::VFIterator<FaceType> vfi(v[1]);
				while (!vfi.End()){
					vfi.V1()->ClearV();
					vfi.V2()->ClearV();
					++vfi;
				}

				// Second Loop
				vfi = face::VFIterator<FaceType>(v[1]);
				while (!vfi.End())
				{
					assert(!vfi.F()->IsD());
					if( !(vfi.V1()->IsV()) && vfi.V1()->IsRW() && vfi.V1()->Cv())
					{
						vfi.V1()->SetV();
						h_ret.push_back(HeapElem(new MYTYPE(VertexPair(vfi.V0(),vfi.V1()), this->GlobalMark(),_pp)));
						std::push_heap(h_ret.begin(),h_ret.end());
						if(!IsSymmetric(pp)){
							h_ret.push_back(HeapElem(new MYTYPE(VertexPair(vfi.V1(),vfi.V0()), this->GlobalMark(),_pp)));
							std::push_heap(h_ret.begin(),h_ret.end());
						}
					}
					if(  !(vfi.V2()->IsV()) && vfi.V2()->IsRW() && vfi.V2()->Cv())
					{
						vfi.V2()->SetV();
						h_ret.push_back(HeapElem(new MYTYPE(VertexPair(vfi.V0(),vfi.V2()),this->GlobalMark(),_pp)));
						std::push_heap(h_ret.begin(),h_ret.end());
						if(!IsSymmetric(pp)){
							h_ret.push_back( HeapElem(new MYTYPE(VertexPair(vfi.V2(),vfi.V0()), this->GlobalMark(),_pp) )  );
							std::push_heap(h_ret.begin(),h_ret.end());
						}
					}
					if(pp->SafeHeapUpdate && vfi.V1()->IsRW() && vfi.V2()->IsRW() )
					{
						h_ret.push_back(HeapElem(new MYTYPE(VertexPair(vfi.V1(),vfi.V2()),this->GlobalMark(),_pp)));
						std::push_heap(h_ret.begin(),h_ret.end());
						if(!IsSymmetric(pp)){
							h_ret.push_back(HeapElem(new MYTYPE(VertexPair(vfi.V2(),vfi.V1()), this->GlobalMark(),_pp)));
							std::push_heap(h_ret.begin(),h_ret.end());
						}
					}

					++vfi;
				}

			}

			static void Init(TriMeshType &m0, TriMeshType &m1, HeapType &h_ret, BaseParameterClass *_pp) {
				QParameter *pp=(QParameter *)_pp;

				typename 	TriMeshType::VertexIterator  vi;
				typename 	TriMeshType::FaceIterator  pf;

				pp->CosineThr=cos(pp->NormalThrRad);

				vcg::tri::UpdateTopology<TriMeshType>::VertexFace(m0);
				vcg::tri::UpdateFlags<TriMeshType>::FaceBorderFromVF(m0);

				vcg::tri::UpdateTopology<TriMeshType>::VertexFace(m1);
				vcg::tri::UpdateFlags<TriMeshType>::FaceBorderFromVF(m1);
/*
				if(pp->FastPreserveBoundary)
				{
					for(pf=m.face.begin();pf!=m.face.end();++pf)
						if( !(*pf).IsD() && (*pf).IsW() )
							for(int j=0;j<3;++j)
								if((*pf).IsB(j))
								{
									(*pf).V(j)->ClearW();
									(*pf).V1(j)->ClearW();
								}
				}

				if(pp->PreserveBoundary)
				{
					WV().clear();
					for(pf=m.face.begin();pf!=m.face.end();++pf)
						if( !(*pf).IsD() && (*pf).IsW() )
							for(int j=0;j<3;++j)
								if((*pf).IsB(j))
								{
									if((*pf).V(j)->IsW())  {(*pf).V(j)->ClearW(); WV().push_back((*pf).V(j));}
									if((*pf).V1(j)->IsW()) {(*pf).V1(j)->ClearW();WV().push_back((*pf).V1(j));}
								}
				}
*/
				InitQuadric(m0,pp);
				InitQuadric(m1,pp);

				TriMeshType &m = m0.VN() < m1.VN() ? m0 : m1;
				// Initialize the heap with all the possible collapses
				if(IsSymmetric(pp))
				{ // if the collapse is symmetric (e.g. u->v == v->u)
					for(vi=m.vert.begin();vi!=m.vert.end();++vi)
						if(!(*vi).IsD() && (*vi).IsRW() && (*vi).Cv())
						{
							vcg::face::VFIterator<FaceType> x;
							for( x.F() = (*vi).VFp(), x.I() = (*vi).VFi(); !x.End(); ++ x){
								x.V1()->ClearV();
								x.V2()->ClearV();
							}
							for (vcg::face::VFIterator<FaceType> x((*vi).Cv()); !x.End(); ++x)
							{
								x.V1()->ClearV();
								x.V2()->ClearV();
							}
							
							for( x.F() = (*vi).VFp(), x.I() = (*vi).VFi(); x.F()!=0; ++x )
							{
								assert(x.F()->V(x.I())==&(*vi));
								if((x.V0()<x.V1()) && x.V1()->IsRW() && !x.V1()->IsV() && x.V1()->Cv()) {
									x.V1()->SetV();
									h_ret.push_back(HeapElem(new MYTYPE(VertexPair(x.V0(),x.V1(),m),TriEdgeCollapse< TriMeshType,VertexPair,MYTYPE>::GlobalMark(),_pp )));
								}
								if((x.V0()<x.V2()) && x.V2()->IsRW()&& !x.V2()->IsV() && x.V2()->Cv()){
									x.V2()->SetV();
									h_ret.push_back(HeapElem(new MYTYPE(VertexPair(x.V0(),x.V2(),m),TriEdgeCollapse< TriMeshType,VertexPair,MYTYPE>::GlobalMark(),_pp )));
								}
							}
						}
				}
				else 
				{ // if the collapse is A-symmetric (e.g. u->v != v->u) 
					for(vi=m.vert.begin();vi!=m.vert.end();++vi)
						if(!(*vi).IsD() && (*vi).IsRW())
						{
							vcg::face::VFIterator<FaceType> x;
							UnMarkAll(m);
							for( x.F() = (*vi).VFp(), x.I() = (*vi).VFi(); x.F()!=0; ++ x)
							{
								assert(x.F()->V(x.I())==&(*vi));
								if(x.V()->IsRW() && x.V1()->IsRW() && !IsMarked(m,x.F()->V1(x.I()))){
									h_ret.push_back( HeapElem( new MYTYPE( VertexPair (x.V(),x.V1()),TriEdgeCollapse< TriMeshType,VertexPair,MYTYPE>::GlobalMark(),_pp)));
								}
								if(x.V()->IsRW() && x.V2()->IsRW() && !IsMarked(m,x.F()->V2(x.I()))){
									h_ret.push_back( HeapElem( new MYTYPE( VertexPair (x.V(),x.V2()),TriEdgeCollapse< TriMeshType,VertexPair,MYTYPE>::GlobalMark(),_pp)));
								}
							}
						}	
				}
			}
		};

	}
}

#endif