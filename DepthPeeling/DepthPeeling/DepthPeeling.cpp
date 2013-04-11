// DepthPeeling.cpp : 定义控制台应用程序的入口点。
//

#include <windows.h>		// Header File For Windows
#include <stdio.h>			// Header File For Standard Input/Output
#include <iostream>
#include <sstream>
#include <gl/glew.h>
#include <gl/gl.h>			// Header File For The OpenGL32 Library
#include <gl/glu.h>			// Header File For The GLu32 Library
#include <gl/glaux.h>		// Header File For The Glaux Library
#include <gl/glut.h>
#include <GLSLProgramObject.h>
#include "glm.h"
#include <nvModel.h>
#include <nvShaderUtils.h>
#include <nvSDKPath.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>


#ifndef M_PI
#define M_PI 3.141592653
#endif

#define RtoD(v) (180.0*(v)/M_PI)
#define DtoR(v) (M_PI*(v)/180.0)

#define GetVertexValue(__buffer, __width, __row, __col) (__buffer[((3*__width)*(__row) + 3*(__col))])
#define GetVertexIndex(__width, __row, __col) ((3*__width)*(__row) + (__col))

static float ZNEAR = 2.0;
static float ZFAR = 8.0;
static float FOVY = 45.0;


//#define MODEL_FILE_PATH "../media/models/Aircraft.obj"
//#define MODEL_FILE_PATH "../media/models/dragon.obj"
#define MODEL_FILE_PATH "../media/models/cow.obj"


using namespace std;

IplImage *_debugImage = NULL;
IplImage *_debugDepthImage = NULL;
int _dbgImageWidth = 512;
int _dbgImageHeight = 512;

unsigned int _windowWidth = 1024;
unsigned int _windowHeight = 768;

int _numOfPasses = 0;

int _windowId[2] = {0};

int g_rotating = 0;
int g_panning = 0;
int g_scaling = 0;
int g_oldX, g_oldY;
int g_newX, g_newY;
float g_fPeelingScale = 1.0;
nv::vec3f g_bbTrans(0.0, 0.0, 0.0);
nv::vec2f g_rot(0.0, 0.0);
nv::vec3f g_pos(0.0, 0.0, 4.0);

nv::vec3f g_up(0.0, -4.0, 0.0);


GLSLProgramObject g_shaderFrontInit;
GLSLProgramObject g_shaderFrontPeel;
GLSLProgramObject g_shaderFrontBlend;
GLSLProgramObject g_shaderFrontSubDepth;
GLSLProgramObject g_shaderFrontFinal;

GLuint g_frontFboId[2];
GLuint g_frontDepthTexId[2];
GLuint g_frontColorTexId[2];
GLuint g_frontColorBlenderTexId;
GLuint g_frontColorBlenderFboId;

#define SAMPLE_ROWS 48
#define SAMPLE_COLS 64
static GLfloat m_aLines[3*2*SAMPLE_COLS*SAMPLE_ROWS] = {0};

const GLint m_nBuffers = 1;
GLfloat *m_pBufferData = NULL;

GLuint *m_pIndices = NULL;
GLuint m_nIndices = 0;

GLuint m_vertexBufferId[m_nBuffers];

const GLint  m_nAttachs = 2;

GLuint g_frontDepthDiffFBOId;
GLuint m_colorAttachTexId[m_nAttachs];
GLuint g_frontDepthDiffDepthTexId;


GLuint g_quadDisplayList;
GLuint g_modelDisplayList;

GLMmodel		* _model;

void LoadModel(char path[])
{
	printf("Loading mesh file...\n");
	_model = glmReadOBJ(path);

	if (_model == NULL)
	{
		fprintf(stderr,"Can't load mesh file: %s\n",path);
		exit(1);
	}
	GLfloat dim[3] = {0};
	glmDimensions(_model, dim);
	float r = sqrtf(dim[0]*dim[0] + dim[1]*dim[1] + dim[2]*dim[2]) / 2;
	float l = sqrtf(g_up.x * g_up.x + g_up.y * g_up.y + g_up.z * g_up.z);
	float r_max = l * sinf(FOVY*M_PI/360);
	g_fPeelingScale = r_max / r;
	//ZNEAR = l - 1.5 * r;
	//ZFAR = l + 1.5 * r;
	
	g_modelDisplayList = glmList(_model, GLM_SMOOTH);
}

void deleteModel()
{
	glmDelete(_model);
	_model = NULL;
	glDeleteLists(g_modelDisplayList, 1);
}


GLfloat* createBuffer()
{
	m_pIndices = (GLuint*)malloc(sizeof(GLuint)*_windowWidth*_windowHeight*6);
	//for (int i=0; i<_windowHeight*_windowWidth;i++)
	//{
	//	m_pIndices[i] = i;
	//}
	unsigned int size = sizeof(GLfloat) * _windowHeight * _windowWidth * 3;
	GLfloat *b = (GLfloat*)malloc(size);
	memset(b,0,size);
	return b;
}

void deleteBuffer(GLfloat *buffer)
{
	free(m_pIndices), m_pIndices = NULL;
	free(buffer);
}

void DrawModel()
{
	if (_model)
		glCallList(g_modelDisplayList);
	else
		glutSolidTeapot(1.f);
}

void DrawDepthLines()
{

}

void InitFrontPeelingRenderTargets()
{
	glGenTextures(2, g_frontDepthTexId);
	glGenTextures(2, g_frontColorTexId);
	glGenFramebuffersEXT(2, g_frontFboId);
	
	for (int i = 0; i < 2; i++)
	{
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, g_frontDepthTexId[i]);
		glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_DEPTH_COMPONENT32F_NV,
			_windowWidth, _windowHeight, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);

		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, g_frontColorTexId[i]);
		glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGBA, _windowWidth, _windowHeight,
			0, GL_RGBA, GL_FLOAT, 0);

		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, g_frontFboId[i]);
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
								  GL_TEXTURE_RECTANGLE_ARB, g_frontDepthTexId[i], 0);
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT,
								  GL_TEXTURE_RECTANGLE_ARB, g_frontColorTexId[i], 0);
	}

	glGenTextures(1,&g_frontDepthDiffDepthTexId);
	glBindTexture(GL_TEXTURE_RECTANGLE_EXT, g_frontDepthDiffDepthTexId);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D,0, GL_DEPTH_COMPONENT32F_NV, _windowWidth, _windowHeight, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);

	glGenTextures(m_nAttachs,m_colorAttachTexId);
	glGenFramebuffersEXT(1, &g_frontDepthDiffFBOId);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, g_frontDepthDiffFBOId);
	for (GLint i=0; i < m_nAttachs; ++i)
	{	
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, m_colorAttachTexId[i]);
		glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGB32F_ARB, _windowWidth, _windowHeight, 0, GL_RGBA, GL_FLOAT, 0);
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT + i,
			GL_TEXTURE_RECTANGLE_ARB, m_colorAttachTexId[i], 0);
	}

	//glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
	//	GL_TEXTURE_RECTANGLE_ARB, g_frontDepthDiffDepthTexId, 0);
}

void DeleteFrontPeelingRenderTargets()
{
	glDeleteFramebuffersEXT(2, g_frontFboId);
	glDeleteTextures(2, g_frontDepthTexId);
	glDeleteTextures(2, g_frontColorTexId);
	glDeleteFramebuffersEXT(1, &g_frontDepthDiffFBOId);
	glDeleteTextures(m_nAttachs, m_colorAttachTexId);
	glDeleteTextures(1, &g_frontDepthDiffDepthTexId);
}

void DoPeeling()
{
	g_frontColorBlenderFboId = g_frontFboId[0];
	g_frontColorBlenderTexId = g_frontColorTexId[0];

	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, g_frontColorBlenderFboId);
	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);

	glClearColor(0.0, 0.0, 0.0, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glEnable(GL_DEPTH_TEST);	// 注意这里开启了深度测试，只有最表层画了出来

	g_shaderFrontInit.bind();
	g_shaderFrontInit.setUniform("scale",&g_fPeelingScale,1);
	DrawModel();	// 画出原始模型，按长度30加上绿色条纹，并将深度值保存在了
					// 帧缓存 g_frontDepthTexId[0] 中。所以下面循环开始时，
					// 表示深度的纹理已经有初始值了。
	g_shaderFrontInit.unbind();
	
	//glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, g_frontColorBlenderFboId);
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0,0,_dbgImageWidth,_dbgImageHeight,GL_BGR,GL_UNSIGNED_BYTE,_debugImage->imageData);
	cvFlip(_debugImage,_debugImage,0);
	cvShowImage("Debug Window",_debugImage);

	glReadPixels(0,0,_dbgImageWidth,_dbgImageHeight,GL_DEPTH_COMPONENT,GL_UNSIGNED_BYTE,_debugDepthImage->imageData);
	cvFlip(_debugDepthImage,_debugDepthImage,0);
	cvShowImage("Depth Window",_debugDepthImage);

	for (int layer = 0; layer < _numOfPasses; layer++) {
		int currId = (layer + 1) % 2;
		int prevId = 1 - currId;

		/********************************************************************

		 ********************************************************************/
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, g_frontFboId[currId]);
		glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);

		glClearColor(0, 0, 0, 1.0);
		glClearDepth(1.0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		
		glDisable(GL_BLEND);
		glEnable(GL_DEPTH_TEST);

		
		g_shaderFrontPeel.bind();	// 这段着色程序中，将比当前最外层还近或相等的片断直接去掉了，
									// 因为当前只画第 N 层，而不是最表层了。
		g_shaderFrontPeel.bindTextureRECT("DepthTex", g_frontDepthTexId[prevId], 0);
		g_shaderFrontPeel.setUniform("scale", &g_fPeelingScale, 1);
		DrawModel();
		g_shaderFrontPeel.unbind();

		/********************************************************************

		 ********************************************************************/
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,g_frontDepthDiffFBOId);
		GLenum att[] = {GL_COLOR_ATTACHMENT0_EXT, GL_COLOR_ATTACHMENT1_EXT};
		glDrawBuffers(1, att);

		glClearColor(0, 0, 0, 1.0);
		glClear(GL_COLOR_BUFFER_BIT );
		glDisable(GL_DEPTH_TEST);

		g_shaderFrontSubDepth.bind();
		g_shaderFrontSubDepth.bindTextureRECT("lastDepth",g_frontDepthTexId[prevId],0);
		g_shaderFrontSubDepth.bindTextureRECT("currentDepth",g_frontDepthTexId[currId],1);
		g_shaderFrontSubDepth.setUniform("zNear", &ZNEAR, 1);
		g_shaderFrontSubDepth.setUniform("zFar", &ZFAR, 1);
		//g_shaderFrontSubDepth.setUniform("zDist", &g_pos.z, 1);
		static GLfloat height = 0.0f;
		g_shaderFrontSubDepth.setUniform("roleHeight", &height, 1);
		glCallList(g_quadDisplayList);
		g_shaderFrontSubDepth.unbind();

		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,_dbgImageWidth,_dbgImageHeight,GL_BGR,GL_UNSIGNED_BYTE,_debugImage->imageData);
		cvFlip(_debugImage,_debugImage,0);
		cvShowImage("Depth Diff Window",_debugImage);

		glReadPixels(0,0,_windowWidth,_windowHeight,GL_RGB,GL_FLOAT,m_pBufferData);
		
		m_nIndices = 0;
		for (unsigned int row=1; row<_windowHeight - 1; ++row)
		{
			for (unsigned int col=1; col<_windowWidth - 1; ++col)
			{
				typedef struct {
					GLfloat x,y,z;
				} _Pos;

				_Pos *p0 = (_Pos *)&GetVertexValue(m_pBufferData, _windowWidth, row - 1, col);
				_Pos *p1 = (_Pos *)&GetVertexValue(m_pBufferData, _windowWidth, row + 0, col);
				_Pos *p2 = (_Pos *)&GetVertexValue(m_pBufferData, _windowWidth, row + 1, col);
				_Pos *tmp[3] = {p0, p1, p2};
				_Pos **vertexs = tmp + 1;
				
				static GLfloat threshold = 0.9 * (ZNEAR - ZFAR) / 2.0;

				for (int i=-1; i <= 1; i++)
				{
					for (int j=-1; j <= 1; j++)
					{
						if (i == 0 && j == 0) continue;
						else if (vertexs[i][j].z < threshold)
							continue;
					}
				}
				
				unsigned int flag = 0;
				flag |= vertexs[0][0].z < threshold?0x1:0;
				flag |= vertexs[0][1].z < threshold?0x2:0;
				flag |= vertexs[1][0].z < threshold?0x4:0;
				flag |= vertexs[1][1].z < threshold?0x8:0;

				if (flag == 0x0) {
					m_pIndices[m_nIndices++] = (row + 0) * _windowWidth + col;
					m_pIndices[m_nIndices++] = (row + 1) * _windowWidth + col;
					m_pIndices[m_nIndices++] = (row + 0) * _windowWidth + col + 1;

					m_pIndices[m_nIndices++] = (row + 0) * _windowWidth + col + 1;
					m_pIndices[m_nIndices++] = (row + 1) * _windowWidth + col;
					m_pIndices[m_nIndices++] = (row + 1) * _windowWidth + col + 1;
				}
				else if (flag == 0x1) {
					m_pIndices[m_nIndices++] = (row + 0) * _windowWidth + col + 1;
					m_pIndices[m_nIndices++] = (row + 1) * _windowWidth + col;
					m_pIndices[m_nIndices++] = (row + 1) * _windowWidth + col + 1;
				}
				else if (flag == 0x2) {
					m_pIndices[m_nIndices++] = (row + 0) * _windowWidth + col;
					m_pIndices[m_nIndices++] = (row + 1) * _windowWidth + col;
					m_pIndices[m_nIndices++] = (row + 1) * _windowWidth + col + 1;
				}
				else if (flag == 0x4) {
					m_pIndices[m_nIndices++] = (row + 0) * _windowWidth + col;
					m_pIndices[m_nIndices++] = (row + 1) * _windowWidth + col + 1;
					m_pIndices[m_nIndices++] = (row + 0) * _windowWidth + col + 1;
				}
				else if (flag == 0x8) {
					m_pIndices[m_nIndices++] = (row + 0) * _windowWidth + col;
					m_pIndices[m_nIndices++] = (row + 1) * _windowWidth + col;
					m_pIndices[m_nIndices++] = (row + 0) * _windowWidth + col + 1;
				}
			}
		}
		
		
		/*
		int stridex = _windowWidth / SAMPLE_COLS;
		int stridey = _windowHeight / SAMPLE_ROWS;
		for (int i=0; i < SAMPLE_ROWS; ++i)
		{
			for (int j=0; j < SAMPLE_COLS; ++j)
			{
				float *data = m_pBufferData + _windowWidth * stridey * i + stridex * j;
				m_aLines[i*3*2 + 0] = data[0];
				m_aLines[i*3*2 + 1] = data[1];
				m_aLines[i*3*2 + 2] = data[2];
			}
		}
		 */

		g_frontColorBlenderTexId = g_frontColorTexId[currId];
		g_frontColorBlenderFboId = g_frontFboId[currId];
	}

	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
	glDrawBuffer(GL_BACK);
	glDisable(GL_DEPTH_TEST);

	glClearColor(0.0, 0.0, 0.0, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	g_shaderFrontFinal.bind();
	g_shaderFrontFinal.bindTextureRECT("finalTex", g_frontColorBlenderTexId,0);
	glCallList(g_quadDisplayList);
	g_shaderFrontFinal.unbind();

}


void BuildShaders()
{
	g_shaderFrontInit.attachVertexShader("init-vertex.glsl");
	g_shaderFrontInit.attachFragmentShader("init-fragment.glsl");
	g_shaderFrontInit.link();

	g_shaderFrontPeel.attachVertexShader("peel-vertex.glsl");
	g_shaderFrontPeel.attachFragmentShader("peel-fragment.glsl");
	g_shaderFrontPeel.link();

	g_shaderFrontBlend.attachVertexShader("blend-vertex.glsl");
	g_shaderFrontBlend.attachFragmentShader("blend-fragment.glsl");
	g_shaderFrontBlend.link();

	g_shaderFrontSubDepth.attachVertexShader("sub-depth-vertex.glsl");
	g_shaderFrontSubDepth.attachFragmentShader("sub-depth-fragment.glsl");
	g_shaderFrontSubDepth.link();

	g_shaderFrontFinal.attachVertexShader("show-vertex.glsl");
	g_shaderFrontFinal.attachFragmentShader("show-fragment.glsl");
	g_shaderFrontFinal.link();
	
}

void MakeFullScreenQuad()
{
	g_quadDisplayList = glGenLists(1);
	glNewList(g_quadDisplayList, GL_COMPILE);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	{
		glLoadIdentity();
		gluOrtho2D(0.0, 1.0, 0.0, 1.0);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();
		glBegin(GL_QUADS);
		{
			glTexCoord2d(0,0);glVertex2f(0.0, 0.0); 
			glTexCoord2d(1,0);glVertex2f(1.0, 0.0);
			glTexCoord2d(1,1);glVertex2f(1.0, 1.0);
			glTexCoord2d(0,1);glVertex2f(0.0, 1.0);
		}
		glEnd();
		glPopMatrix();
		glMatrixMode(GL_PROJECTION);
	}
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);

	glEndList();
}

void initGL()
{
	InitFrontPeelingRenderTargets();

	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

	BuildShaders();
	LoadModel(MODEL_FILE_PATH);
	MakeFullScreenQuad();

	glDisable(GL_CULL_FACE);
	glDisable(GL_LIGHTING);
	glDisable(GL_NORMALIZE);

	glClearColor (0.0, 0.0, 0.0, 0.0);
	glShadeModel(GL_SMOOTH);

	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	glDisable(GL_DITHER);
}



void displayPeelingView()
{
	glutSetWindow(_windowId[0]);
	//static double s_t0 = currentSeconds();
	glClearColor(0,0,0,0);
	glClearDepth(1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	GLfloat ratio = g_fPeelingScale * _windowWidth / _windowHeight;
	glOrtho(-ratio, ratio, -g_fPeelingScale, g_fPeelingScale, ZNEAR, ZFAR);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	
	gluLookAt(g_up.x, g_up.y, g_up.z, 0, 0, 0, 0, 0, 1);
	
	glScalef(g_fPeelingScale*2, g_fPeelingScale*2, g_fPeelingScale*2);

	DoPeeling();
	
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glutSwapBuffers();
}

void displayCameraView()
{
	glutSetWindow(_windowId[1]);
	//static double s_t0 = currentSeconds();
	glClearColor(0,0,0,0);
	glClearDepth(1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	gluLookAt(g_pos.x, g_pos.y, g_pos.z, g_pos.x, g_pos.y, 0, 0, 1, 0);
	//glTranslatef(0, 0, -5);

	glRotatef(g_rot.x, 1, 0, 0);
	glRotatef(g_rot.y, 0, 1, 0);
	glTranslatef(g_bbTrans.x, g_bbTrans.y, g_bbTrans.z);

	float r = ZFAR - ZNEAR;
	float l = sqrtf(g_up.x * g_up.x + g_up.y * g_up.y + g_up.z * g_up.z);
	float r_max = l * sinf(FOVY*M_PI/360);
    float g_fCameraScale = r_max / r * 2;

	glScalef(g_fCameraScale, g_fCameraScale, g_fCameraScale);

	glColor3f(1.0, 1.0, 1.0);
	glutWireCube(1.0);

	glEnableClientState(GL_VERTEX_ARRAY);

	glVertexPointer(3, GL_FLOAT, 0, m_pBufferData);;
	glColor3f(1.0, 0.0, 0.0);
	glDrawElements(GL_TRIANGLES, m_nIndices, GL_UNSIGNED_INT, m_pIndices);

	glDisableClientState(GL_VERTEX_ARRAY);

	glutSwapBuffers();
}

void idle()
{
	glutSetWindow(_windowId[1]);
	glutPostRedisplay();
}


void reshape(int w, int h)
{
	
	if (_windowWidth != w || _windowHeight != h) {
		_windowWidth = w;
		_windowHeight = h;
		DeleteFrontPeelingRenderTargets();
		InitFrontPeelingRenderTargets();
		deleteBuffer(m_pBufferData);
		m_pBufferData = createBuffer();
	}

	int window = glutGetWindow();
	if (window == _windowId[0]) {
		glutSetWindow(_windowId[1]);
		glutReshapeWindow(_windowWidth, _windowHeight);
	}
	else {
		glutSetWindow(_windowId[0]);
		glutReshapeWindow(_windowWidth, _windowHeight);
	}

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(FOVY, (float)_windowWidth/(float)_windowHeight, 0.02, 50);

	glMatrixMode(GL_MODELVIEW);
	glViewport(0, 0, _windowWidth, _windowHeight);
}

void keyboardFunc(unsigned char key, int x, int y)
{
	key = (unsigned char)tolower(key);
	switch(key)
	{
	case '+':case '=':
		_numOfPasses++;
		glutSetWindow(_windowId[0]);
		glutPostRedisplay();
		break;
	case '-':case '_':
		if (_numOfPasses > 0) {
			_numOfPasses--;
			glutSetWindow(_windowId[0]);
			glutPostRedisplay();
		}
		break;
	case 'r':case 'R':
		g_pos = nv::vec3f(0,0,4);
		g_rot = nv::vec2f(0,0);
		g_bbTrans = nv::vec3f(0,0,0);
		break;;
	case 27:
		exit(0);
		break;
	}
	glutSetWindow(_windowId[1]);
	glutPostRedisplay();
}

void timerFunc(int value)
{
	
	glutTimerFunc(1000.0/60,timerFunc,value + 1);
}

void motionFunc(int x, int y)
{
	g_oldX = g_newX;
	g_oldY = g_newY;

	g_newX = x;
	g_newY = y;

	float rel_x = (g_newX - g_oldX) / (float)_windowWidth;
	float rel_y = (g_newY - g_oldY) / (float)_windowHeight;
	if (g_rotating)
	{
		g_rot.y += (rel_x * 180);
		g_rot.x += (rel_y * 180);
	}
	else if (g_panning)
	{
		g_pos.x -= 4. * rel_x;
		g_pos.y += 4. * rel_y;
	}
	else if (g_scaling)
	{
		g_pos.z -= rel_y * g_pos.z;
		//g_bbScale -= rel_y * g_bbScale;
	}

	glutPostRedisplay();
}

void mouseFunc(int button, int state, int x, int y)
{
	g_newX = x; g_newY = y;
	fprintf(stdout,"btn: %d, state: %d\n",button,state);
	switch(button)
	{
	case GLUT_LEFT_BUTTON:
		g_rotating = !state;
		break;
	case GLUT_MIDDLE_BUTTON:
		g_panning = !state;
		break;
	case GLUT_RIGHT_BUTTON:
		g_scaling = !state;
		break;
	}
}


int main(int argc, char* argv[])
{
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowSize(_windowWidth, _windowHeight);
	glutInit(&argc, argv);
	_windowId[0] = glutCreateWindow("Peeling View");
	glutDisplayFunc(displayPeelingView);
	glutReshapeFunc(reshape);

	//glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	//glutInitWindowSize(_windowWidth, _windowHeight);
	//_windowId[1] = glutCreateWindow("Show");

	if (glewInit() != GLEW_OK)
	{
		printf("glewInit failed. Exiting...\n");
		exit(1);
	}

	if (!glewIsSupported( "GL_VERSION_2_0 "
		"GL_ARB_texture_rectangle "
		"GL_ARB_texture_float "
		"GL_NV_float_buffer "
		"GL_NV_depth_buffer_float "))
	{
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

	initGL();
	m_pBufferData = createBuffer();

	_debugImage = cvCreateImage(cvSize(_dbgImageWidth,_dbgImageHeight),8,3);
	_debugDepthImage = cvCreateImage(cvSize(_dbgImageWidth,_dbgImageHeight),8,1);

	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowSize(_windowWidth, _windowHeight);
	glutInit(&argc, argv);
	_windowId[1] = glutCreateWindow("Camera View");

	glutDisplayFunc(displayCameraView);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboardFunc);
	glutMouseFunc(mouseFunc);
	glutMotionFunc(motionFunc);
	//glutTimerFunc(1000.0/60,timerFunc,0);
	glutIdleFunc(idle);

	glutMainLoop();

	cvReleaseImage(&_debugImage);
	cvReleaseImage(&_debugDepthImage);
	cvDestroyAllWindows();

	deleteBuffer(m_pBufferData);

	return 0;
}
