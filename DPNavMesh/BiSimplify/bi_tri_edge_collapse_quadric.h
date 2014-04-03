
#ifndef __BI_TRI_EDGE_COLLAPSE_QUARDIC_H
#define __BI_TRI_EDGE_COLLAPSE_QUARDIC_H


#include <vcg/complex/algorithms/local_optimization.h>
#include <vcg/complex/algorithms/local_optimization/tri_edge_collapse_quadric.h>
#include <eigenlib/Eigen/Dense>
#include <eigenlib/Eigen/LU>

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
            inline BiVertexPair( VertexType *v0, VertexType *v1): BasicVertexPair<VertexType>(v0, v1), m(NULL) {}
            inline BiVertexPair( VertexType *v0, VertexType *v1, MeshType &mesh): BasicVertexPair<VertexType>(v0, v1), m(&mesh) {}
            inline BiVertexPair( VertexType *v0, VertexType *v1, MeshType *mesh): BasicVertexPair<VertexType>(v0, v1), m(mesh) {}
            static BiVertexPair CorresVertexPair(const BiVertexPair &p) {return BiVertexPair(p.cV(0)->Cv(), p.cV(1)->Cv(), p.m->Cm());}
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

            BiTriEdgeCollapse(const VertexPair &p, int i, vcg::BaseParameterClass *pp): TriEdgeCollapseQuadric<TriMeshType, VertexPair, MYTYPE>(p,i,pp){}
            /*
            计算使总Error 最小的两个对应点的位置
            */
            void ComputeBiMinimal(CoordType &This, CoordType &Other)
            {
                typename TriMeshType::VertexType * v[2], * v_corres[2];
                v[0] = this->pos.V(0);
                v[1] = this->pos.V(1);
                v_corres[0] = v[0]->Cv();
                v_corres[1] = v[1]->Cv();

                QuadricType q=QH::Qd(v[0]);
                q+=QH::Qd(v[1]);

                QuadricType p=QH::Qd(v_corres[0]);
                p+=QH::Qd(v_corres[1]);

                using namespace Eigen;

                Matrix4d A;
                Vector4d b;
                A << p.a[0]+q.a[0],p.a[1]+q.a[1],q.a[2],p.a[2],

                     p.a[1]+q.a[1],p.a[3]+q.a[3],q.a[4],p.a[4],

                     q.a[2]       ,q.a[4]       ,q.a[5],0,

                     p.a[2]       ,p.a[4]       ,0     ,p.a[5];

                b << -(p.b[0]+q.b[0])/2,
                     -(p.b[1]+q.b[1])/2,
                     -q.b[2]/2,
                     -p.b[2]/2;
                if (fabs(A.determinant()) > std::numeric_limits<double>::epsilon()) {
                    Vector4d result = A.inverse()*b;
                    This[0] = Other[0] = result[0];
                    This[1] = Other[1] = result[1];
                    This[2] = result[2];
                    Other[2] = result[3];
                }
                else {

                    bool rt0=q.Minimum(This);
                    bool rt1=p.Minimum(Other);
                    if(!rt0 && !rt1) { // if the computation of the minimum fails we choose between the two edge points and the middle one.
                        This.Import((v[0]->cP()+v[1]->cP())/2);
                        Other.Import((v_corres[0]->cP()+v_corres[1]->cP())/2);
                        double qvx=q.Apply(This)+p.Apply(Other);
                        double qv0=q.Apply(v[0]->cP())+p.Apply(v_corres[0]->cP());
                        double qv1=q.Apply(v[1]->cP())+p.Apply(v_corres[1]->cP());
                        if(qv0<qvx) {
                            This = v[0]->cP();
                            Other = v_corres[0]->cP();
                        }
                        if(qv1<qvx && qv1<qv0) {
                            This = v[1]->cP();
                            Other = v_corres[1]->cP();
                        }
                    }
                    else if (!rt1) {
                        Other = This;
                        Other[2] = (v_corres[0]->cP()[2]+v_corres[1]->cP()[2])/2;
                    }
                    else {
                        This = Other;
                        This[2] = (v[0]->cP()[2]+v[1]->cP()[2])/2;
                    }
                }



                //return CoordType::Construct(x);
            }

            ScalarType ComputePriority(vcg::BaseParameterClass *_pp)
            {
                QParameter *pp=(QParameter *)_pp;
                ScalarType error;
                typename vcg::face::VFIterator<FaceType> x;
                std::vector<CoordType> on; // original normals
                typename TriMeshType::VertexType * v[2];
                v[0] = this->pos.V(0);
                v[1] = this->pos.V(1);

                if(pp->NormalCheck){ // Compute maximal normal variation
                    // store the old normals for non-collapsed face in v0
                    for(x.F() = v[0]->VFp(), x.I() = v[0]->VFi(); x.F()!=0; ++x )	 // for all faces in v0		
                        if(x.F()->V(0)!=v[1] && x.F()->V(1)!=v[1] && x.F()->V(2)!=v[1] ) // skip faces with v1
                            on.push_back(NormalizedNormal(*x.F()));
                    // store the old normals for non-collapsed face in v1
                    for(x.F() = v[1]->VFp(), x.I() = v[1]->VFi(); x.F()!=0; ++x )	 // for all faces in v1	
                        if(x.F()->V(0)!=v[0] && x.F()->V(1)!=v[0] && x.F()->V(2)!=v[0] ) // skip faces with v0
                            on.push_back(NormalizedNormal(*x.F()));
                }

                //// Move the two vertexe  into new position (storing the old ones)
                CoordType OldPos0=v[0]->cP();
                CoordType OldPos1=v[1]->cP();
                CoordType OldPosOther0 = v[0]->Cv()->cP();
                CoordType OldPosOther1 = v[1]->Cv()->cP();

                if(pp->OptimalPlacement) {
                    ComputeBiMinimal(v[0]->P(), v[0]->Cv()->P());
                    v[1]->P()=v[0]->P();
                    v[1]->Cv()->P()=v[0]->Cv()->P();
                }
                else {
                    v[0]->P() = v[1]->P();
                    v[0]->Cv()->P() = v[1]->Cv()->P();
                }

                //// Rescan faces and compute quality and difference between normals
                int i;
                double ndiff,MinCos  = std::numeric_limits<double>::max(); // minimo coseno di variazione di una normale della faccia 
                // (e.g. max angle) Mincos varia da 1 (normali coincidenti) a
                // -1 (normali opposte);
                double qt,   MinQual = std::numeric_limits<double>::max();
                CoordType nn;
                for(x.F() = v[0]->VFp(), x.I() = v[0]->VFi(),i=0; x.F()!=0; ++x ) {	// for all faces in v0		
                    if(x.F()->V(0)!=v[1] && x.F()->V(1)!=v[1] && x.F()->V(2)!=v[1] )		// skip faces with v1
                    {
                        if(pp->NormalCheck){
                            nn=NormalizedNormal(*x.F());
                            ndiff=nn.dot(on[i++]);
                            if(ndiff<MinCos) MinCos=ndiff;
                        }
                        if(pp->QualityCheck){
                            qt= QualityFace(*x.F());
                            if(qt<MinQual) MinQual=qt;
                        }
                    }
                }
                for(x.F() = v[1]->VFp(), x.I() = v[1]->VFi()/*,i=0*/; x.F()!=0; ++x ) {		// for all faces in v1	
                    if(x.F()->V(0)!=v[0] && x.F()->V(1)!=v[0] && x.F()->V(2)!=v[0] )			// skip faces with v0
                    {
                        if(pp->NormalCheck){
                            nn=NormalizedNormal(*x.F());
                            ndiff=nn.dot(on[i++]);
                            if(ndiff<MinCos) MinCos=ndiff;
                        }
                        if(pp->QualityCheck){
                            qt= QualityFace(*x.F());
                            if(qt<MinQual) MinQual=qt;
                        }
                    }
                }
                QuadricType qq=QH::Qd(v[0]);
                qq+=QH::Qd(v[1]);
                Point3d tpd=Point3d::Construct(v[1]->P());
                double QuadErr = pp->ScaleFactor*qq.Apply(tpd);

                QuadricType p = QH::Qd(v[0]->Cv());
                p += QH::Qd(v[1]->Cv());
                QuadErr += pp->ScaleFactor*p.Apply(v[1]->Cv()->cP());

                // All collapses involving triangles with quality larger than <QualityThr> has no penalty;
                if(MinQual > pp->QualityThr)
                    MinQual = pp->QualityThr;

                if(pp->NormalCheck) {
                    // All collapses where the normal vary less  than <NormalThr> (e.g. more than CosineThr)
                    // have no penalty
                    if(MinCos>pp->CosineThr)
                        MinCos=pp->CosineThr;
                    MinCos=(MinCos+1)/2.0; // Now it is in the range 0..1 with 0 very dangerous!
                }

                if(QuadErr<pp->QuadricEpsilon) QuadErr=pp->QuadricEpsilon;

                if( pp->UseVertexWeight ) QuadErr *= (QH::W(v[1])+QH::W(v[0]))/2;

                if(!pp->QualityCheck && !pp->NormalCheck) error = (ScalarType)(QuadErr);
                if( pp->QualityCheck && !pp->NormalCheck) error = (ScalarType)(QuadErr / MinQual);
                if(!pp->QualityCheck &&  pp->NormalCheck) error = (ScalarType)(QuadErr / MinCos);
                if( pp->QualityCheck &&  pp->NormalCheck) error = (ScalarType)(QuadErr / (MinQual*MinCos));

                //Rrestore old position of v0 and v1
                v[0]->P()=OldPos0;
                v[1]->P()=OldPos1;
                v[0]->Cv()->P() = OldPosOther0;
                v[1]->Cv()->P() = OldPosOther1;

                return error;
            }

            void Execute(TriMeshType &m, vcg::BaseParameterClass *_pp) {
                assert(this->pos.m == &m);

                QParameter *pp=(QParameter *)_pp;
                CoordType newPos0, newPos1;

                VertexPairType vp0, vp1;
                vp0 = this->pos;
                vp1 = VertexPairType(vp0.V(0)->Cv(), vp0.V(1)->Cv(), vp0.m->Cm());

                if(pp->OptimalPlacement) {
                    //CoordType p0=vp0.V(0)->cP();
                    //CoordType p1=vp0.V(1)->cP();
                    //printf("Position:%6.6f %6.6f %6.6f, %6.6f %6.6f %6.6f\n", p0[0], p0[1], p0[2], p1[0], p1[1], p1[2]);
                    ComputeBiMinimal(newPos0, newPos1);
                    //printf("New position:%6.6f %6.6f %6.6f\n", newPos0[0], newPos0[1], newPos0[2]);
                    /*
                    newPos0 = static_cast<MYTYPE*>(this)->ComputeMinimal();
                    this->pos = vp1;
                    newPos1 = static_cast<MYTYPE*>(this)->ComputeMinimal();
                    */
                }
                else {
                    newPos0=this->pos.V(1)->P();
                    newPos1=this->pos.V(1)->Cv()->P();
                }
                //newPos1[0] = newPos0[0];
                //newPos1[1] = newPos0[1];

                QH::Qd(vp0.V(1))+=QH::Qd(vp0.V(0));
                EdgeCollapser<TriMeshType,VertexPairType>::Do(*vp0.m, vp0, newPos0); // v0 is deleted and v1 take the new position
                QH::Qd(vp1.V(1))+=QH::Qd(vp1.V(0));
                EdgeCollapser<TriMeshType,VertexPairType>::Do(*vp1.m, vp1, newPos1); // v0 is deleted and v1 take the new position
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
                //int t=0;
                while (!vfi.End()){
                    if (!vfi.F()->IsD()) {
                        vfi.V1()->ClearV();
                        vfi.V2()->ClearV();
                        vfi.V1()->Cv()->ClearV();
                        vfi.V2()->Cv()->ClearV();
                    }
                    ++vfi;
                    //++t;
                }
                //printf("num of faces %d\n",t);

                // Second Loop
                vfi = face::VFIterator<FaceType>(v[1]);
                while (!vfi.End())
                {
                    assert(!vfi.F()->IsD());
                    if( !(vfi.V1()->IsV()) && vfi.V1()->IsRW() && vfi.V1()->Cv())
                    {
                        vfi.V1()->SetV();
                        h_ret.push_back(HeapElem(new MYTYPE(VertexPair(vfi.V0(),vfi.V1(),this->pos.m), this->GlobalMark(),_pp)));
                        std::push_heap(h_ret.begin(),h_ret.end());
                        if(!IsSymmetric(pp)){
                            h_ret.push_back(HeapElem(new MYTYPE(VertexPair(vfi.V1(),vfi.V0(),this->pos.m), this->GlobalMark(),_pp)));
                            std::push_heap(h_ret.begin(),h_ret.end());
                        }
                    }
                    if(  !(vfi.V2()->IsV()) && vfi.V2()->IsRW() && vfi.V2()->Cv())
                    {
                        vfi.V2()->SetV();
                        h_ret.push_back(HeapElem(new MYTYPE(VertexPair(vfi.V0(),vfi.V2(),this->pos.m),this->GlobalMark(),_pp)));
                        std::push_heap(h_ret.begin(),h_ret.end());
                        if(!IsSymmetric(pp)){
                            h_ret.push_back( HeapElem(new MYTYPE(VertexPair(vfi.V2(),vfi.V0(),this->pos.m), this->GlobalMark(),_pp) )  );
                            std::push_heap(h_ret.begin(),h_ret.end());
                        }
                    }
                    if(pp->SafeHeapUpdate && vfi.V1()->IsRW() && vfi.V2()->IsRW() )
                    {
                        h_ret.push_back(HeapElem(new MYTYPE(VertexPair(vfi.V1(),vfi.V2(),this->pos.m),this->GlobalMark(),_pp)));
                        std::push_heap(h_ret.begin(),h_ret.end());
                        if(!IsSymmetric(pp)){
                            h_ret.push_back(HeapElem(new MYTYPE(VertexPair(vfi.V2(),vfi.V1(),this->pos.m), this->GlobalMark(),_pp)));
                            std::push_heap(h_ret.begin(),h_ret.end());
                        }
                    }

                    ++vfi;
                }

            }

            static void Init(TriMeshType &m, HeapType &h_ret, BaseParameterClass *_pp) {
                assert(m.Cm()); // Mesh must have a corresponding mesh!
                QParameter *pp=(QParameter *)_pp;

                typename 	TriMeshType::VertexIterator  vi;
                typename 	TriMeshType::FaceIterator  pf;

                pp->CosineThr=cos(pp->NormalThrRad);

                vcg::tri::UpdateTopology<TriMeshType>::VertexFace(m);
                vcg::tri::UpdateFlags<TriMeshType>::FaceBorderFromVF(m);

                vcg::tri::UpdateTopology<TriMeshType>::VertexFace(*m.Cm());
                vcg::tri::UpdateFlags<TriMeshType>::FaceBorderFromVF(*m.Cm());

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

                InitQuadric(m,pp);
                InitQuadric(*m.Cm(),pp);

                // Initialize the heap with all the possible collapses
                if(IsSymmetric(pp))
                { // if the collapse is symmetric (e.g. u->v == v->u)
                    for(vi=m.vert.begin();vi!=m.vert.end();++vi)
                        if(!(*vi).IsD() && (*vi).IsRW() && (*vi).Cv())
                        {
                            for(vcg::face::VFIterator<FaceType> x(&*vi); !x.End(); ++ x){
                                x.V1()->ClearV();
                                x.V2()->ClearV();
                                x.V1()->Cv()->ClearV();
                                x.V2()->Cv()->ClearV();
                            }

                            for(vcg::face::VFIterator<FaceType> x(&*vi); !x.End(); ++ x)
                            {
                                assert(x.F()->V(x.I())==&(*vi));
                                if((x.V0()<x.V1()) && x.V1()->IsRW() && !x.V1()->IsV() && x.V1()->Cv()) {
                                    x.V1()->SetV();
                                    x.V1()->Cv()->SetV();
                                    h_ret.push_back(HeapElem(new MYTYPE(VertexPair(x.V0(),x.V1(),m),TriEdgeCollapse< TriMeshType,VertexPair,MYTYPE>::GlobalMark(),_pp )));
                                }
                                if((x.V0()<x.V2()) && x.V2()->IsRW() && !x.V2()->IsV() && x.V2()->Cv()){
                                    x.V2()->SetV();
                                    x.V2()->Cv()->SetV();
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