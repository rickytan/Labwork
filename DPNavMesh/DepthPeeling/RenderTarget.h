#pragma once

#include <gl/glew.h>

class RenderTarget
{
public:
    RenderTarget();
    virtual ~RenderTarget(void);
    void generate(int width, int height);
    void destroy();
    void bindFrameBuffer(int index);
    void readBuffer(int width, int height, GLenum format, GLenum type, GLvoid *buffer);

//private:
    GLuint m_frameBufferObjects[2];
    GLuint m_depthTextures[2];
    GLuint m_colorTextures[2];

};
