#include <cstdio>
#include <cstdlib>
#include <ctime>

#include <gl/glew.h>
#include <gl/glut.h>

#include <vcg/complex/complex.h>
#include <eigenlib/Eigen/Dense>

#include "GLSLProgram.h"
#include "Timer.h"

#pragma warning(disable:4819)

#ifdef _DEBUG
#pragma comment(lib, "glew32.lib")
#else
#pragma comment(lib, "glew32.lib")
#endif

static int g_windowWidth = 800, g_windowHeight = 600;
static float FOVY = 60.0f, ZNEAR = 1.0f, ZFAR = 10.0f;
static int g_mainWindow = 0;
static GLfloat g_modelScale = 1.0;
static double g_globalTime = 0.0;
static int g_mouseX = 0, g_mouseY = 0;

using namespace Eigen;

static Eigen::Vector3d g_eyeCenter(0.f, 0.f, 0.f);
static Eigen::Vector3d g_eyePosition(0.f, 0.f, 5.f);
static Eigen::Vector3d g_eyeUp(0.f, 1.f, 0.f);

static const int MAX_PEELING_LEVEL = 8;

static int g_currentLevel = 0;

GLint g_FBOIds[MAX_PEELING_LEVEL] = {0};
GLint g_TextureIds[MAX_PEELING_LEVEL] = {0};


GLSLProgram g_shaderFront;


void initShader()
{
    g_shaderFront.addVertexShader("init-vertex.glsl");
    g_shaderFront.addFragmentShader("init-fragment.glsl");
    g_shaderFront.linkProgram();

}

void initLight()
{


}

void init()
{
    initShader();

    //glDisable(GL_CULL_FACE);
    glDisable(GL_NORMALIZE);
    glEnable(GL_CULL_FACE);
    

    initLight();

    glViewport(0, 0, g_windowWidth, g_windowHeight);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(FOVY, (double)g_windowWidth/g_windowHeight, ZNEAR, ZFAR);
    //glOrtho(-1, 1, -1, 1, -1, 10);
}

void drawModel()
{
    glColor3f(0, 1.0, 0.0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glBegin(GL_QUADS);
    glVertex3f(-1, -1, 0);
    glVertex3f(-1,  1, 0);
    glVertex3f( 1,  1, 0);
    glVertex3f( 1, -1, 0);
    glEnd();
    glutSolidTeapot(2.0);
    glutSolidSphere(2.0, 64, 32);
}

void doPeeling()
{
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
        g_eyePosition.x(), g_eyePosition.y(), g_eyePosition.z(),
        g_eyeCenter.x(), g_eyeCenter.y(), g_eyeCenter.z(),
        g_eyeUp.x(), g_eyeUp.y(), g_eyeUp.z()
        );
    glScalef(g_modelScale, g_modelScale, g_modelScale);

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

            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            gluPerspective(FOVY, (double)g_windowWidth/g_windowHeight, ZNEAR, ZFAR);
            glMatrixMode(GL_MODELVIEW);
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

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    //glutTimerFunc(1000/60, timer, 0);
    glutIdleFunc(idle);
    

    glutMainLoop();

    return 0;
}