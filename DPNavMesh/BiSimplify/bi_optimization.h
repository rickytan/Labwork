
#ifndef __BI_OPTIMIZATION
#define __BI_OPTIMIZATION

#include<vector>
#include<algorithm>
#include<time.h>
#include<math.h>
#include<vcg/complex/complex.h>
#include <vcg/space/index/kdtree/kdtree.h>
#include <vcg/complex/algorithms/local_optimization.h>

namespace vcg{

	template<class MeshType>
	class Vertex2dConstDataWrapper :public ConstDataWrapper<typename MeshType::CoordType>
	{
	public:
		inline Vertex2dConstDataWrapper(MeshType &m):
		ConstDataWrapper<typename MeshType::CoordType> ( NULL, m.vert.size(), sizeof(typename MeshType::CoordType))
		{
			DataType *data = new DataType(m.vert.size());
			for (in i=0; i<m.VN();++i)
			{
				data[i][0] = m.vert[i][0];
				data[i][1] = m.vert[i][1];
				data[i][2] = 0;
			}
			mpData = data;
		}
		~Vertex2dConstDataWrapper() {delete mpData;}
	};


	template<class MeshType>
	class BiOptimization
	{
	public:
		BiOptimization(MeshType &_m0, MeshType &_m1, BaseParameterClass *_pp): m0(_m0), m1(_m1) { ClearTermination();e=0.0;HeapSimplexRatio=5; pp=_pp;}

		typedef LocalOptimization::HeapElem HeapElem;
		// scalar type
		typedef typename MeshType::ScalarType ScalarType;
		// coord type
		typedef typename MeshType::CoordType CoodType;
		// type of the heap
		typedef typename std::vector<HeapElem> HeapType;	
		// modification type	
		typedef  LocalModification <MeshType>  LocModType;
		// modification Pointer type	
		typedef  LocalModification <MeshType> * LocModPtrType;

		/// termination conditions	
		typedef LocalOptimization::LOTermination LOTermination;

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

		void SetTargetSimplices	(int ts			){nTargetSimplices	= ts;	SetTerminationFlag(LOnSimplices);	}	 	
		void SetTargetVertices	(int tv			){nTargetVertices	= tv;	SetTerminationFlag(LOnVertices);	} 
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
		MeshType & m0, & m1;

		HeapType h0, h1;

		/// Default distructor
		~BiOptimization(){ 
			typename HeapType::iterator i;
			for(i = h0.begin(); i != h0.end(); i++)
				delete (*i).locModPtr;
			for (i = h1.begin(); i != h1.end(); ++i)
			{
				delete (*i).locModPtr;
			}

		};

		double e;

		/// main cycle of optimization
		bool DoOptimization()
		{
			start=clock();
			nPerfmormedOps =0;
			while( !GoalReached() && !h0.empty())
			{
				if(h0.size()> m.SimplexNumber()*HeapSimplexRatio )  ClearHeap();
				std::pop_heap(h0.begin(),h0.end());
				LocModPtrType  locMod   = h0.back().locModPtr;
				currMetric=h0.back().pri;
				h0.pop_back();

				if( locMod->IsUpToDate() )
				{	
					//printf("popped out: %s\n",locMod->Info(m));
					// check if it is feasible
					if (locMod->IsFeasible(this->pp))
					{
						nPerfmormedOps++;
						locMod->Execute(m,this->pp);
						locMod->UpdateHeap(h0,this->pp);
					}
				}
				//else printf("popped out unfeasible\n");
				delete locMod;
			}
			return !(h0.empty());
		}

		// It removes from the heap all the operations that are no more 'uptodate' 
		// (e.g. collapses that have some recently modified vertices)
		// This function  is called from time to time by the doOptimization (e.g. when the heap is larger than fn*3)
		void ClearHeap()
		{
			typename HeapType::iterator hi;
			//int sz=h.size();
			for(hi=h0.begin();hi!=h0.end();)
			{
				if(!(*hi).locModPtr->IsUpToDate())
				{
					delete (*hi).locModPtr;
					*hi=h0.back();
					if(&*hi==&h0.back()) 
					{
						hi=h0.end();
						h0.pop_back();
						break;
					}
					h0.pop_back();
					continue;
				}
				++hi;
			}
			//qDebug("\nReduced heap from %7i to %7i (fn %7i) ",sz,h.size(),m.fn);
			make_heap(h0.begin(),h0.end());
		}

		///initialize for all vertex the temporary mark must call only at the start of decimation
		///by default it takes the first element in the heap and calls Init (static funcion) of that type
		///of local modification. 
		template <class LocalModificationType> void Init()
		{
			vcg::tri::InitVertexIMark(m0);
			vcg::tri::InitVertexIMark(m1);

			// The expected size of heap depends on the type of the local modification we are using..
			HeapSimplexRatio = LocalModificationType::HeapSimplexRatio(pp);

			LocalModificationType::Init(m0,h0,pp);
			LocalModificationType::Init(m1,h1,pp);

			std::make_heap(h0.begin(),h0.end());
			std::make_heap(h1.begin(),h1.end());

			if(!h0.empty())
				currMetric=h0.front().pri;
		}


		template <class LocalModificationType> void Finalize()
		{
			LocalModificationType::Finalize(m,h0,pp);
		}


		/// say if the process is to end or not: the process ends when any of the termination conditions is verified
		/// override this function to implemetn other tests
		bool GoalReached(){
			assert ( ( ( tf & LOnSimplices	)==0) ||  ( nTargetSimplices!= -1));
			assert ( ( ( tf & LOnVertices	)==0) ||  ( nTargetVertices	!= -1));
			assert ( ( ( tf & LOnOps		)==0) ||  ( nTargetOps		!= -1));
			assert ( ( ( tf & LOMetric		)==0) ||  ( targetMetric	!= -1));
			assert ( ( ( tf & LOTime		)==0) ||  ( timeBudget		!= -1));

			if ( IsTerminationFlag(LOnSimplices) &&	( m.SimplexNumber()<= nTargetSimplices)) return true;
			if ( IsTerminationFlag(LOnVertices)  &&  ( m.VertexNumber() <= nTargetVertices)) return true;
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
			for(hi=h0.begin();hi!=h0.end();++hi)
				if(!(*hi).locModPtr->IsUpToDate())
				{
					*hi=h0.back();
					h0.pop_back();
					if(hi==h0.end()) break;
				}
				//printf("\nReduced heap from %i to %i",sz,h.size());
				make_heap(h0.begin(),h0.end());
		}

	private:
		void FindCorresponding()
		{
			Vertex2dConstDataWrapper<MeshType> dw(m0);
			KdTree<ScalarType> tree(dw);
			tree.setMaxNofNeighbors(1);

			for (int i=0;i<m1.VN();++i)
			{
				CoordType location = m1.vert[i].cP();
				location[2] = 0;
				tree.doQueryK(location);
				int neighbor = tree.getNeighborId(0);

				m1.vert[i].Cv() = &(m0.vert[neighbor]);
				m0.vert[neighbor].Cv() = &(m1.vert[i]);
			}
			
		}

	};//end class decimation

}//end namespace
#endif
