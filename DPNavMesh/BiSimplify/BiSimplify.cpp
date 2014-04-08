// BiSimplify.cpp : 定义控制台应用程序的入口点。
//

#pragma warning(disable:4819)
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "mesh.h"
//#include "bi_optimization.h"

#include <gl/glut.h>

#ifndef MIN
#define MIN(x, y) ((x)<(y)?(x):(y))
#endif

static GLint win_width = 800, win_height = 600;
static GLfloat fovy = 45.0f;
static GLfloat near = 0.1f, far = 500.0f;

Mesh mesh;

void Simplify(
	__in const Mesh &mesh_in,
	__out Mesh &mesh_out,
	__in const int final_face = 1000,
	__in const float quality = 0.3f)
{
	

}

void init()
{
	glDisable(GL_LIGHTING);
	glEnable(GL_CULL_FACE);
	
}

void display()
{
	glClearColor(0, 0, 0, 0);
	glClearDepth(1.0);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glColor3f(1.0, 1.0, 1.0);
	glutWireTeapot(1.0);

	gluLookAt(
		0, 0, 10,
		0, 0, 0, 
		0, 1, 0);
	glutSwapBuffers();
}

void idle()
{
	glutPostRedisplay();
}

void reshape(int width, int height)
{
	if (width != win_width || height != win_height) {
		win_height = height;
		win_width = width;

		glutReshapeWindow(width, height);

		GLfloat ratio = 4.0 / 2;
		if (height > 0)
			ratio = (GLfloat)width / height;

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(fovy, ratio, near, far);

		glMatrixMode(GL_MODELVIEW);
		glViewport(0, 0, width, height);

	}
}

void gui(int *argc, char *argv[])
{
	glutInit(argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
	init();
	glutInitWindowPosition(0, 0);
	glutInitWindowSize(win_width, win_height);
	glutCreateWindow("Show");
	
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutIdleFunc(idle);
	
	glutShowWindow();
	glutMainLoop();
}

void load_mesh(Mesh &mesh, const char *mesh_file)
{
	printf("Loading mesh: %s...\n", mesh_file);
	int errcode = vcg::tri::io::Importer<Mesh>::Open(mesh, mesh_file);
	if (errcode) {
		printf("Unable to load mesh %s: %s\n", mesh_file, vcg::tri::io::Importer<Mesh>::ErrorMsg(errcode));
		exit(1);
	}
	printf("Mesh loaded with Vertex: %d\tFace: %d\n\n", mesh.vn, mesh.fn);
}

void genMesh(Mesh &mesh)
{
	srand(time(NULL));

	int mesh_size = 16;
	Mesh::VertexIterator vi = vcg::tri::Allocator<Mesh>::AddVertices(mesh, (mesh_size) * (mesh_size) * 4);
	Mesh::FaceIterator fi = vcg::tri::Allocator<Mesh>::AddFaces(mesh, mesh_size * mesh_size * 2);
	//Mesh::VertexPointer vp[mesh_size + 1];
	for (int i=0;i<mesh_size;++i)
	{
		for (int j=0;j<mesh_size;++j)
		{
			Mesh::VertexPointer vp[4] = {0};
			vp[0] = &*vi++;
			vp[1] = &*vi++;
			vp[2] = &*vi++;
			vp[3] = &*vi++;

			vp[0]->P() = Mesh::CoordType(i + 0 + 0.01 * ((rand()-5)%10), j + 0 + 0.01 * ((rand()-5)%10), 0);
			vp[1]->P() = Mesh::CoordType(i + 0 + 0.01 * ((rand()-5)%10), j + 1 + 0.01 * ((rand()-5)%10), 0);
			vp[2]->P() = Mesh::CoordType(i + 1 + 0.01 * ((rand()-5)%10), j + 1 + 0.01 * ((rand()-5)%10), 0);
			vp[3]->P() = Mesh::CoordType(i + 1 + 0.01 * ((rand()-5)%10), j + 0 + 0.01 * ((rand()-5)%10), 0);

			fi->V(0) = vp[0];
			fi->V(1) = vp[1];
			fi->V(2) = vp[2];
			++fi;
			fi->V(0) = vp[2];
			fi->V(1) = vp[3];
			fi->V(2) = vp[0];
			++fi;
		}
	}
}

void test()
{
	const char *mesh_file0 = "single_removed0.ply", *mesh_file1 = "single_removed1.ply";
	int final_size = 1000;
	Mesh m0, m1;
	
	load_mesh(m0, mesh_file0);
	load_mesh(m1, mesh_file1);

	m0.Cm() = &m1;
	m1.Cm() = &m0;

	vcg::tri::BiTriEdgeCollapseQuadricParameter<Mesh::CoordType> params;
	params.QualityThr = .3;

	vcg::tri::UpdateBounding<Mesh>::Box(m0);
	vcg::tri::UpdateBounding<Mesh>::Box(m1);

	vcg::BiOptimization<Mesh> bioptim(m0, &params);
	bioptim.Init<MyTriEdgeCollapse>();
	bioptim.SetTargetVertices(final_size);
    bioptim.SetTimeBudget(1.0);
	
	while(bioptim.DoOptimization() && bioptim.corresVnum > final_size) {
        printf("Corresponding Vertics %10d\r", bioptim.corresVnum);
	}
	vcg::tri::io::Exporter<Mesh>::Save(m0, "m0.ply");
	vcg::tri::io::Exporter<Mesh>::Save(m1, "m1.ply");
}

using namespace std;

int main(int argc, char* argv[])
{
	test();
	return 0;

	Mesh m;
	genMesh(m);
	vcg::tri::io::Exporter<Mesh>::Save(m, "generated.ply");
	return 0;
    
	char *mesh_file	= "face_mesh0.ply";
	char *out_file	= "face_mesh0_simp.ply";
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
	//optim.SetTargetSimplices(final_size);
	optim.SetTargetVertices(final_size);
	optim.SetTimeBudget(0.5f);

	while (optim.DoOptimization() && mesh.fn > final_size)
		printf("Mesh size %7i \r", mesh.fn);

	vcg::tri::io::Exporter<Mesh>::Save(mesh, out_file);

	return 0;
}

