#include <gl/glew.h>
#include <gl/glut.h>


static int g_windowWidth = 800, g_windowHeight = 600;
static float FOVY = 60.0f, ZNEAR = 1.0f, ZFAR = 10.0f;
static int g_mainWindow = 0;

void init()
{

}

void display()
{

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

void reshape(int width, int height)
{
    if (g_windowWidth != width ||
        g_windowHeight != height) {
            g_windowWidth = width;
            g_windowHeight = height;


    }
}

int main(int argc, char *argv[])
{
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
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
    glutInitWindowSize(g_windowWidth, g_windowHeight);
    g_mainWindow = glutCreateWindow("Depth Peeling");
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);

    return 0;
}