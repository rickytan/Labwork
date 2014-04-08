
#ifndef __SINGLE_VERTEX_REMOVER
#define __SINGLE_VERTEX_REMOVER

#include<vector>
#include<algorithm>
#include<time.h>
#include<math.h>
#include<vcg/complex/complex.h>

template <typename TriMeshType>
class SingleVertexRemover
{
    typedef typename vcg::face::VFIterator<typename TriMeshType::FaceType> VFI;
    typedef typename TriMeshType::VertexIterator VI;
    typedef std::vector<typename TriMeshType::VertexType *> VertexPointContainer;
    typedef typename VertexPointContainer::iterator VPCI;
public:
    static int Remove(TriMeshType &mesh) {
        vcg::tri::UpdateTopology<TriMeshType>::VertexFace(mesh);
        int deletedVextex = 0;
        VertexPointContainer ver_to_delete;
        for (VI vi=mesh.vert.begin();vi!=mesh.vert.end();++vi)
        {
            if (!vi->IsD() && vi->IsR()) {
                int fn = 0;
                for (VFI vfi(&*vi);!vfi.End();++vfi,++fn)
                {
                    if (fn > 0)
                        break;
                }
                if (fn <= 0 && vi->IsW()) {  // Single Vertex, has no face arround it
                    ver_to_delete.push_back(&*vi);
                }
            }
        }
        for (VPCI v = ver_to_delete.begin(); v != ver_to_delete.end(); ++v)
            vcg::tri::Allocator<TriMeshType>::DeleteVertex(mesh, **v);
        return ver_to_delete.size();
    }
};
#endif  // end of __SINGLE_VERTEX_REMOVER