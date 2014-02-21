// BiSimplify.cpp : 定义控制台应用程序的入口点。
//

#pragma warning(disable:4819)
#include "mesh.h"


int main(int argc, char* argv[])
{
	Mesh mesh;

	char *mesh_file	= "bunny_closed.obj";
	char *out_file	= "bunny_out.obj";
	int final_size	= 1000;

	printf("Loading %s\n", mesh_file);
	int error = vcg::tri::io::Importer<Mesh>::Open(mesh, mesh_file);
	if(error)
	{
		printf("Unable to open mesh %s : '%s'\n", mesh_file, vcg::tri::io::Importer<Mesh>::ErrorMsg(error));
		exit(-1);
	}
	printf("Mesh loaded with Vertex: %d\tFace: %d\n\n", mesh.vn, mesh.fn);

	vcg::tri::TriEdgeCollapseQuadricParameter params;
	params.QualityThr = .3;

	vcg::tri::UpdateBounding<Mesh>::Box(mesh);

	vcg::LocalOptimization<Mesh> optim(mesh, &params);
	optim.Init<TriEdgeCollapse>();
	optim.SetTargetSimplices(final_size);
	optim.SetTimeBudget(0.5f);

	while (optim.DoOptimization() && mesh.fn > final_size)
		printf("Mesh size %7i \r", mesh.fn);

	vcg::tri::io::Exporter<Mesh>::Save(mesh, out_file);

	return 0;
}

