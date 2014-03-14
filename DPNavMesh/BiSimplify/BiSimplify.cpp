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

void test()
{
	const char *mesh_file0 = "face_mesh0.ply", *mesh_file1 = "face_mesh1.ply";
	int final_size = 1000;
	Mesh m0, m1;
	
	load_mesh(m0, mesh_file0);
	load_mesh(m1, mesh_file1);

	vcg::tri::TriEdgeCollapseQuadricParameter params;
	params.QualityThr = .3;

	vcg::tri::UpdateBounding<Mesh>::Box(m0);
	vcg::tri::UpdateBounding<Mesh>::Box(m1);

	vcg::BiOptimization<Mesh> bioptim(m0, m1, &params);
	bioptim.Init<MyTriEdgeCollapse>();
	bioptim.SetTargetSimplices(final_size);
	
	//while(bioptim.DoOptimization() && 
	//	(m0.fn > final_size || m1.fn > final_size)) {

	//}
}

int main(int argc, char* argv[])
{
	test();
	return 0;

	char *mesh_file	= "bunny_closed.obj";
	char *out_file	= "bunny_out.obj";
	int final_size	= 100;

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

