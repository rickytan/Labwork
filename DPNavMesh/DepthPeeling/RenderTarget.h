#pragma once

#include <gl/glew.h>

class RenderTarget
{
public:
    RenderTarget();
    ~RenderTarget(void);
    void generate(int width, int height);
    void destroy();
    void bindFrameBuffer(int index);
    void readColor(int width, int height, GLenum format, GLenum type, GLvoid *buffer);
    void readVertex(int width, int height, GLenum format, GLenum type, GLvoid *buffer);
    void readDepth(int width, int height, GLvoid *buffer);
//private:
    GLuint m_frameBufferObjects[2];
    GLuint m_depthTextures[2];
    GLuint m_colorTextures[2];
    GLuint m_vertexTextures[2];
    
    int m_currentIdx;
};
