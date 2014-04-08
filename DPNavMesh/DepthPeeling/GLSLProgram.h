#pragma once

#include <gl/glew.h>
#include <string>
#include <vector>

using namespace std;

class GLSLProgram
{
public:
    GLSLProgram(void);
    ~GLSLProgram(void);

    void destroy();
    void use() { glUseProgram(m_progId); }
    void unuse() { glUseProgram(0); }
    void setUniform(std::string name, GLfloat *val, int count);
    void setTexture(std::string texname, GLint texunit);
    void addVertexShader(std::string vertfile);
    void addFragmentShader(std::string fragfile);
    void linkProgram();
    bool isValid() { return !!glIsProgram(m_progId); }
    GLint getProgram() { return m_progId; }

private:
    vector<GLint> m_vertShaders, m_fragShaders;
    GLint m_progId;
};
