#include <cstdio>

#include "RenderTarget.h"

RenderTarget::RenderTarget(void)
{
}

RenderTarget::~RenderTarget(void)
{
    destroy();
}

void RenderTarget::generate(int width, int height)
{
    glGenFramebuffersEXT(2, m_frameBufferObjects);
    glGenTextures(2, m_depthTextures);
    glGenTextures(2, m_colorTextures);

    for (int i=0;i<2;++i)
    {
        glBindTexture(GL_TEXTURE_RECTANGLE_ARB, m_depthTextures[i]);
        glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_DEPTH_COMPONENT32F_NV,
            width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);

        glBindTexture(GL_TEXTURE_RECTANGLE_ARB, m_colorTextures[i]);
        glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGBA, width, height,
            0, GL_RGBA, GL_FLOAT, 0);

        glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, m_frameBufferObjects[i]);
        glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
            GL_TEXTURE_RECTANGLE_ARB, m_depthTextures[i], 0);
        glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT,
            GL_TEXTURE_RECTANGLE_ARB, m_colorTextures[i], 0);
    }
}

void RenderTarget::destroy()
{
    glDeleteFramebuffersEXT(2, m_frameBufferObjects);
    glDeleteTextures(2, m_depthTextures);
    glDeleteTextures(2, m_colorTextures);
}

void RenderTarget::bindFrameBuffer(int index)
{
    m_currentIdx = index;
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, m_frameBufferObjects[index]);
    glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
}

void RenderTarget::readBuffer(int width, int height, GLenum format, GLenum type, GLvoid *buffer)
{
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, m_frameBufferObjects[m_currentIdx]);
    glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
    glReadPixels(0, 0, width, height, format, type, buffer);
}

void RenderTarget::readDepth(int width, int height, GLvoid *buffer)
{
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, m_frameBufferObjects[m_currentIdx]);
    glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
    glReadPixels(0, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT, buffer);
}