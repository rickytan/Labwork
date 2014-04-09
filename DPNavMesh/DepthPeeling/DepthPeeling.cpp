
#pragma warning(disable:4819)
#pragma warning(disable:4244)

#include <cstdio>
#include <cstdlib>
#include <ctime>

#include "Mesh.h"
#include "GLSLProgram.h"
#include "Timer.h"

#include <gl/glew.h>
#include <gl/glut.h>


#ifdef _DEBUG
#pragma comment(lib, "glew32.lib")
#else
#pragma comment(lib, "glew32.lib")
#endif

static int g_windowWidth = 800, g_windowHeight = 600;
static float FOVY = 60.0f, ZNEAR = 1.0f, ZFAR = 10.0f;
static int g_mainWindow = 0;
static GLfloat g_modelScale = 1.0;
static vcg::Point3f g_modelTranslate(0, 0, 0);
static double g_globalTime = 0.0;
static int g_mouseX = 0, g_mouseY = 0;


static vcg::Point3f g_eyeCenter(0.f, 0.f, 0.f);
static vcg::Point3f g_eyePosition(0.f, 0.f, 5.f);
static vcg::Point3f g_eyeUp(0.f, 1.f, 0.f);

static const int MAX_PEELING_LEVEL = 8;

static int g_currentLevel = 0;

GLint g_FBOIds[MAX_PEELING_LEVEL] = {0};
GLint g_TextureIds[MAX_PEELING_LEVEL] = {0};


GLSLProgram g_shaderFront;

Mesh g_mesh;
vcg::GlTrimesh<Mesh> g_model;

void initShader()
{
    g_shaderFront.addVertexShader("init-vertex.glsl");
    g_shaderFront.addFragmentShader("init-fragment.glsl");
    g_shaderFront.linkProgram();

}

void initLight()
{


}

void setupProjection()
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    //gluPerspective(FOVY, (double)g_windowWidth/g_windowHeight, ZNEAR, ZFAR);
    GLfloat ratio = 1.0 * g_windowWidth / g_windowHeight;
    if (ratio > 1.0)
        glOrtho(-ratio, ratio, -1, 1, ZNEAR, ZFAR);
    else
        glOrtho(-1, 1, -1.0 / ratio, 1.0 / ratio, ZNEAR, ZFAR);
}

void init()
{
    initShader();

    //glDisable(GL_CULL_FACE);
    glDisable(GL_NORMALIZE);
    glEnable(GL_CULL_FACE);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    initLight();

    glViewport(0, 0, g_windowWidth, g_windowHeight);
    setupProjection();
}

void loadModel(std::string filename)
{
    cout << "Loading: " << filename << endl;
    vcg::tri::io::Importer<Mesh>::Open(g_mesh, filename.c_str());
    cout << "Mesh loaded with vertex: " << g_mesh.VN() << ", face: " << g_mesh.FN() << endl;
    vcg::tri::UpdateBounding<Mesh>::Box(g_mesh);
    vcg::tri::UpdateTopology<Mesh>::VertexFace(g_mesh);
    g_modelTranslate = -g_mesh.bbox.Center();
    g_modelScale = 2.0 / g_mesh.bbox.Diag();
    g_model.m = &g_mesh;
}

void drawModel()
{
    glColor3f(0, 1.0, 0.0);
    glPushMatrix();
    glTranslatef(g_modelTranslate.X(), g_modelTranslate.Y(), g_modelTranslate.Z());
    glScalef(g_modelScale, g_modelScale, g_modelScale);
    g_model.Draw<vcg::GLW::DMFlat, vcg::GLW::CMNone, vcg::GLW::TMNone>();
    glPopMatrix();
}

void doPeeling()
{
    glEnable(GL_DEPTH_TEST);

    g_shaderFront.use();
    g_shaderFront.setUniform("scale", &g_modelScale, 1);
    drawModel();
    g_shaderFront.unuse();
}

void display()
{
    glClearColor(0, 0, 0, 0);
    glClearDepth(1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(
        g_eyePosition.X(), g_eyePosition.Y(), g_eyePosition.Z(),
        g_eyeCenter.X(), g_eyeCenter.Y(), g_eyeCenter.Z(),
        g_eyeUp.X(), g_eyeUp.Y(), g_eyeUp.Z()
        );

    doPeeling();

    glutSwapBuffers();
}

void keyboard(unsigned char key, int x, int y)
{
    switch(key) {
        case 27:
            exit(0);
            break;
        default:
            break;
    }
}

void motion(int x, int y)
{
    g_mouseX = x;
    g_mouseY = y;
}

void mouse(int button, int state, int x, int y)
{
    printf("Mouse: button %d, state %d, x %d, y %d\n", button, state, x, y);
    g_mouseX = x;
    g_mouseY = y;
    switch(button) {
        case GLUT_LEFT_BUTTON:
            break;
        case GLUT_RIGHT_BUTTON:
            break;
        case GLUT_MIDDLE_BUTTON:
            break;
    }
}

void reshape(int width, int height)
{
    if (g_windowWidth != width ||
        g_windowHeight != height) {
            g_windowWidth = width;
            g_windowHeight = height;

            glViewport(0, 0, g_windowWidth, g_windowHeight);

            setupProjection();
    }
}

void idle()
{
    g_globalTime = currentSeconds();
    glutPostRedisplay();
}

void timer(int value)
{
    g_globalTime = currentSeconds();
    glutPostRedisplay();
    glutTimerFunc(1000/60, timer, 0);
}

int main(int argc, char *argv[])
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
    glutInitWindowSize(g_windowWidth, g_windowHeight);
    g_mainWindow = glutCreateWindow("Depth Peeling");

    if (glewInit() != GLEW_OK) {
        printf("glewInit failed!\n");
        exit(1);
    }

    if (!glewIsSupported(
        "GL_VERSION_2_0 "
        "GL_ARB_texture_rectangle "
        "GL_ARB_texture_float "
        "GL_NV_float_buffer "
        "GL_NV_depth_buffer_float ")) {
        printf("Unable to load the necessary extensions\n");
        printf("This sample requires:\n");
        printf("OpenGL version 2.0\n");
        printf("GL_ARB_texture_rectangle\n");
        printf("GL_ARB_texture_float\n");
        printf("GL_NV_float_buffer\n");
        printf("GL_NV_depth_buffer_float\n");
        printf("Exiting...\n");
        exit(1);
    }

    init();
    loadModel("bunny_closed.obj");

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    //glutTimerFunc(1000/60, timer, 0);
    //glutIdleFunc(idle);
    

    glutMainLoop();

    return 0;
}