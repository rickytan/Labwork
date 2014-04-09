
#pragma warning(disable:4819)
#pragma warning(disable:4244)

#include <cstdio>
#include <cstdlib>
#include <ctime>

#include "Mesh.h"
#include "GLSLProgram.h"
#include "RenderTarget.h"
#include "Timer.h"

#include <gl/glew.h>
#include <gl/glut.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>


#ifdef _DEBUG
#pragma comment(lib, "glew32.lib")
#pragma comment(lib, "cv200d.lib")
#pragma comment(lib, "cxcore200d.lib")
#pragma comment(lib, "highgui200d.lib")
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
GLuint g_depthTexture;

GLSLProgram g_shaderFront;
GLSLProgram g_shaderPeeling;

RenderTarget g_renderTarget;

Mesh g_mesh;
vcg::GlTrimesh<Mesh> g_model;

void initShader()
{
    g_shaderFront.addVertexShader("init-vertex.glsl");
    g_shaderFront.addFragmentShader("init-fragment.glsl");
    g_shaderFront.linkProgram();

    g_shaderPeeling.addVertexShader("peel-vertex.glsl");
    g_shaderPeeling.addFragmentShader("peel-fragment.glsl");
    g_shaderPeeling.linkProgram();
}

void createBuffer()
{
    g_renderTarget.generate(g_windowWidth, g_windowHeight);

    glGenTextures(1, &g_depthTexture);
}

void deleteBuffer()
{
    g_renderTarget.destroy();
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
    initLight();
    createBuffer();

    //glDisable(GL_CULL_FACE);
    glDisable(GL_NORMALIZE);
    glEnable(GL_CULL_FACE);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);



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
    vcg::Point3f vec = g_mesh.bbox.max - g_mesh.bbox.min;
    // a simple way
    //g_modelScale = 1.0 / max(vec[0], max(vec[1], vec[2])) / tanf(FOVY / 2 * M_PI / 180);

    // a another
    vcg::Point3f eyeDir = (g_eyeCenter - g_eyePosition).Normalize();
    float dis = max((vcg::Point3f(vec[0], 0, 0)^eyeDir).Norm(), max((vcg::Point3f(0, vec[1], 0)^eyeDir).Norm(), (vcg::Point3f(0, 0, vec[2])^eyeDir).Norm()));
    g_modelScale = 1.0 / dis / tanf(FOVY / 2 * M_PI / 180);
    g_model.m = &g_mesh;
}

void drawModel()
{
    glColor3f(0, 1.0, 0.0);
    glPushMatrix();
    glScalef(g_modelScale, g_modelScale, g_modelScale);
    glTranslatef(g_modelTranslate.X(), g_modelTranslate.Y(), g_modelTranslate.Z());
    g_model.Draw<vcg::GLW::DMFlat, vcg::GLW::CMLast, vcg::GLW::TMNone>();
    glPopMatrix();
    glutSolidTeapot(.5);
}

void doPeeling()
{
    g_renderTarget.bindFrameBuffer(0);

    glEnable(GL_DEPTH_TEST);

    g_shaderFront.use();
    g_shaderFront.setUniform("scale", &g_modelScale, 1);
    drawModel();
    g_shaderFront.unuse();

    IplImage *image = cvCreateImage(cvSize(g_windowWidth, g_windowHeight), 8, 3);
    g_renderTarget.readBuffer(g_windowWidth, g_windowHeight, GL_BGR, GL_UNSIGNED_BYTE, image->imageData);
    cvFlip(image, image);
    cvShowImage("Debug", image);
    cvReleaseImage(&image);

    for (int i=0; i < g_currentLevel; ++i)
    {
        int currId = (i + 1) % 2;
        int prevId = 1 - currId;

        g_renderTarget.bindFrameBuffer(currId);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glDisable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);

        g_shaderPeeling.use();
        g_shaderPeeling.setTexture("DepthTex", 0, GL_TEXTURE_RECTANGLE_ARB, g_depthTexture);
        drawModel();
        g_shaderPeeling.unuse();
    }
    
    glBindFramebuffer(GL_FRAMEBUFFER_EXT, 0);
    glDrawBuffer(GL_BACK);
    glEnable(GL_TEXTURE);
    glBindTexture(GL_TEXTURE0, g_renderTarget.m_colorTextures[0]);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, 1, 0, 1, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glBegin(GL_QUADS);
    glTexCoord2f(0, 0);
    glVertex2f(0, 0);
    glTexCoord2f(0, 1);
    glVertex2f(0, 1);
    glTexCoord2f(1, 1);
    glVertex2f(1, 1);
    glTexCoord2f(1, 0);
    glVertex2f(1, 0);
    glEnd();
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
        case '-':
        case '_':
            if (g_currentLevel > 0) {
                --g_currentLevel;
                glutPostRedisplay();
            }
            break;
        case '+':
        case '=':
            ++g_currentLevel;
            glutPostRedisplay();
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

void printGLInfo()
{
    fprintf(stdout, "OpenGL info:\n"
        "{\n"
        "\tExtensions: %s\n"
        "\tVendor: %s\n"
        "\tRenderer: %s\n"
        "\tVersion: %s\n"
        "\tShading Language: %s\n"
        "}\n", glGetString(GL_EXTENSIONS), glGetString(GL_VENDOR), glGetString(GL_RENDERER), glGetString(GL_VERSION), glGetString(GL_SHADING_LANGUAGE_VERSION));
}

int main(int argc, char *argv[])
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
    glutInitWindowSize(g_windowWidth, g_windowHeight);
    g_mainWindow = glutCreateWindow("Depth Peeling");

    printGLInfo();

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
        printf(
            "Unable to load the necessary extensions\n"
            "This sample requires:\n"
            "OpenGL version 2.0\n"
            "GL_ARB_texture_rectangle\n"
            "GL_ARB_texture_float\n"
            "GL_NV_float_buffer\n"
            "GL_NV_depth_buffer_float\n"
            "Exiting...\n"
            );
        exit(1);
    }

    init();
    loadModel("casa.ply");

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