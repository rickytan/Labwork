
#ifndef __BI_OPTIMIZATION
#define __BI_OPTIMIZATION

#include<vector>
#include<algorithm>
#include<time.h>
#include<math.h>
#include<vcg/complex/complex.h>
#include <vcg/space/index/kdtree/kdtree.h>
#include <vcg/complex/algorithms/local_optimization.h>
#include "single_vertex_remover.h"

namespace vcg{

	template<class MeshType>
	class Vertex2dConstDataWrapper :public ConstDataWrapper<typename MeshType::CoordType>
	{
	public:
		inline Vertex2dConstDataWrapper(MeshType &m):
		ConstDataWrapper<typename MeshType::CoordType> ( NULL, m.vert.size(), sizeof(typename MeshType::CoordType))
		{
			DataType *data = new DataType[m.vert.size()];
			unsigned int idx = 0;
			for (MeshType::VertexIterator vi = m.vert.begin(); vi != m.vert.end(); ++vi, ++idx)
			{
				data[idx] = vi->cP();
				data[idx][2] = 0.0;
			}
			mpData = reinterpret_cast<const unsigned char*>(data);
		}
		~Vertex2dConstDataWrapper() {delete[] mpData;}

	};


	template<class MeshType>
	class BiOptimization
	{
	public:
		BiOptimization(MeshType &_m, BaseParameterClass *_pp): m(_m), corresVnum(0), corresFnum(0) { ClearTermination();e=0.0;HeapSimplexRatio=5; pp=_pp;}

		typedef typename LocalOptimization<MeshType>::HeapElem HeapElem;
		// scalar type
		typedef typename MeshType::ScalarType ScalarType;
		// coord type
		typedef typename MeshType::CoordType CoordType;
		// type of the heap
		typedef typename std::vector<HeapElem> HeapType;	
		// modification type	
		typedef  LocalModification <MeshType>  LocModType;
		// modification Pointer type	
		typedef  LocalModification <MeshType> * LocModPtrType;

		/// termination conditions	
		enum LOTermination {	
			LOnCorresSimplices	= 0x01,	// test number of simplicies	
			LOnCorresVertices	= 0x02, // test number of verticies
			LOnOps				= 0x04, // test number of operations
			LOMetric			= 0x08, // test Metric (error, quality...instance dependent)
			LOTime				= 0x10  // test how much time is passed since the start
		} ;

		int tf;

		int nPerfmormedOps,
			nTargetOps,
			nTargetSimplices,
			nTargetVertices;

		float	timeBudget;
		clock_t	start;
		ScalarType currMetric;
		ScalarType targetMetric;
		BaseParameterClass *pp;

		// The ratio between Heap size and the number of simplices in the current mesh
		// When this value is exceeded a ClearHeap Start;

		float HeapSimplexRatio; 

		void SetTerminationFlag		(int v){tf |= v;}
		void ClearTerminationFlag	(int v){tf &= ~v;}
		bool IsTerminationFlag		(int v){return ((tf & v)!=0);}

		void SetTargetSimplices	(int ts			){nTargetSimplices	= ts;	SetTerminationFlag(LOnCorresSimplices);	}	 	
		void SetTargetVertices	(int tv			){nTargetVertices	= tv;	SetTerminationFlag(LOnCorresVertices);	} 
		void SetTargetOperations(int to			){nTargetOps		= to;	SetTerminationFlag(LOnOps);			} 

		void SetTargetMetric	(ScalarType tm	){targetMetric		= tm;	SetTerminationFlag(LOMetric);		} 
		void SetTimeBudget		(float tb		){timeBudget		= tb;	SetTerminationFlag(LOTime);			} 

		void ClearTermination()
		{
			tf=0;
			nTargetSimplices=0;
			nTargetOps=0;
			targetMetric=0;
			timeBudget=0;
			nTargetVertices=0;
		}
		/// the two corresponding meshes to optimize
		MeshType & m;

		HeapType h;

		/// Default distructor
		~BiOptimization(){ 
			typename HeapType::iterator i;
			for(i = h.begin(); i != h.end(); i++)
				delete (*i).locModPtr;
		};

		double e;

		/// main cycle of optimization
		bool DoOptimization()
		{
			start=clock();
			nPerfmormedOps =0;
			while( !GoalReached() && !h.empty())
			{
				if(h.size()> corresVnum*HeapSimplexRatio ) 
					ClearHeap();
				std::pop_heap(h.begin(),h.end());
				LocModPtrType locMod	= h.back().locModPtr;
				currMetric				= h.back().pri;
				h.pop_back();

				if( locMod->IsUpToDate() )
				{	
					//printf("popped out: %s\n",locMod->Info(m));
					// check if it is feasible
					if (locMod->IsFeasible(this->pp))
					{
						nPerfmormedOps++;
						--corresVnum;
						locMod->Execute(m,this->pp);
						locMod->UpdateHeap(h, this->pp);
					}
				}
				//else printf("popped out unfeasible\n");
				delete locMod;
			}
			return !(h.empty());
		}

		// It removes from the heap all the operations that are no more 'uptodate' 
		// (e.g. collapses that have some recently modified vertices)
		// This function  is called from time to time by the doOptimization (e.g. when the heap is larger than fn*3)
		void ClearHeap()
		{
			typename HeapType::iterator hi;
			//int sz=h.size();
			for(hi=h.begin();hi!=h.end();)
			{
				if(!(*hi).locModPtr->IsUpToDate())
				{
					delete (*hi).locModPtr;
					*hi=h.back();
					if(&*hi==&h.back()) 
					{
						hi=h.end();
						h.pop_back();
						break;
					}
					h.pop_back();
					continue;
				}
				++hi;
			}
			//qDebug("\nReduced heap from %7i to %7i (fn %7i) ",sz,h.size(),m.fn);
			make_heap(h.begin(),h.end());
		}

		typedef std::pair<typename MeshType::VertexType *, typename MeshType::VertexType *> CorresPair;
		typedef std::vector<CorresPair> CorresPairContainer;

		int corresVnum, corresFnum;

		///initialize for all vertex the temporary mark must call only at the start of decimation
		///by default it takes the first element in the heap and calls Init (static funcion) of that type
		///of local modification. 
		template <class LocalModificationType>
		void Init(bool aligned = true)
		{
			assert(m.Cm());
			if (!aligned) {

			}
            vcg::tri::UpdateTopology<MeshType>::VertexFace(m);
            vcg::tri::UpdateFlags<MeshType>::FaceBorderFromVF(m);

            vcg::tri::UpdateTopology<MeshType>::VertexFace(*m.Cm());
            vcg::tri::UpdateFlags<MeshType>::FaceBorderFromVF(*m.Cm());

            int d=0;
            d = SingleVertexRemover<MeshType>::Remove(m);
            printf("%d single vertex deleted!\n",d);
            d = SingleVertexRemover<MeshType>::Remove(*m.Cm());
            printf("%d single vertex deleted!\n",d);

			CorresPairContainer cpc;
			FindCorresponding(cpc);

			vcg::tri::InitVertexIMark(m);
			vcg::tri::InitVertexIMark(*m.Cm());

			// The expected size of heap depends on the type of the local modification we are using..
			HeapSimplexRatio = LocalModificationType::HeapSimplexRatio(pp);

			LocalModificationType::Init(m,h,pp);

			std::make_heap(h.begin(),h.end());

			if(!h.empty())
				currMetric=h.front().pri;
		}


		template <class LocalModificationType>
		void Finalize()
		{
			LocalModificationType::Finalize(m,h,pp);
		}


		/// say if the process is to end or not: the process ends when any of the termination conditions is verified
		/// override this function to implemetn other tests
		bool GoalReached(){
			assert ( ( ( tf & LOnCorresSimplices	)==0) ||  ( nTargetSimplices!= -1));
			assert ( ( ( tf & LOnCorresVertices	)==0) ||  ( nTargetVertices	!= -1));
			assert ( ( ( tf & LOnOps		)==0) ||  ( nTargetOps		!= -1));
			assert ( ( ( tf & LOMetric		)==0) ||  ( targetMetric	!= -1));
			assert ( ( ( tf & LOTime		)==0) ||  ( timeBudget		!= -1));

			if ( IsTerminationFlag(LOnCorresSimplices) &&	( corresFnum<= nTargetSimplices)) return true;
			if ( IsTerminationFlag(LOnCorresVertices)  &&  ( corresVnum <= nTargetVertices)) return true;
			if ( IsTerminationFlag(LOnOps)		   && (nPerfmormedOps	== nTargetOps)) return true;
			if ( IsTerminationFlag(LOMetric)		 &&  ( currMetric		> targetMetric)) return true;
			if ( IsTerminationFlag(LOTime) )
			{
				clock_t cur = clock();
				if(cur<start) // overflow of tick counter;
					return true; // panic
				else
					if ( (cur - start)/(double)CLOCKS_PER_SEC > timeBudget) return true;
			}
			return false;
		}



		///erase from the heap the operations that are out of date
		void ClearHeapOld()
		{
			typename HeapType::iterator hi;
			for(hi=h.begin();hi!=h.end();++hi)
				if(!(*hi).locModPtr->IsUpToDate())
				{
					*hi=h.back();
					h.pop_back();
					if(hi==h.end())
                        break;
				}
				//printf("\nReduced heap from %i to %i",sz,h.size());
				make_heap(h.begin(),h.end());
		}

	private:
		void FindCorresponding(CorresPairContainer &cpc)
		{
			Vertex2dConstDataWrapper<MeshType> dw(m);
			KdTree<ScalarType> tree(dw);
			tree.setMaxNofNeighbors(1);

			for (unsigned int i=0;i<m.Cm()->vert.size();++i)
			{
				CoordType location = m.Cm()->vert[i].cP();
				location[2] = 0;
				tree.doQueryK(location);
				int neighbor = tree.getNeighborId(0);

				const CoordType loc = m.vert[neighbor].cP();
				if (fabs(loc[0] - location[0]) < std::numeric_limits<ScalarType>::epsilon() &&
					fabs(loc[1] - location[1]) < std::numeric_limits<ScalarType>::epsilon()) {
						m.Cm()->vert[i].Cv() = &(m.vert[neighbor]);
						m.vert[neighbor].Cv() = &(m.Cm()->vert[i]);
						++corresVnum;
				}
			}
		}

	};//end class decimation

}//end namespace
#endif
