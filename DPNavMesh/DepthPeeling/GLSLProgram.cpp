#include "GLSLProgram.h"
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>


GLSLProgram::GLSLProgram(void)
: m_progId(0)
, m_vertShaders()
, m_fragShaders()
{
}

GLSLProgram::~GLSLProgram(void)
{
    destroy();
}

void GLSLProgram::destroy()
{
    for (vector<GLint>::iterator it=m_vertShaders.begin();it!=m_vertShaders.end();++it)
    {
        glDeleteShader(*it);
    }

    for (vector<GLint>::iterator it=m_fragShaders.begin();it!=m_fragShaders.end();++it)
    {
        glDeleteShader(*it);
    }

    if (m_progId) {
        glDeleteProgram(m_progId);
    }
}

void GLSLProgram::setUniform(std::string name, GLfloat *val, int count)
{
    GLint linked;
    glGetProgramiv(m_progId, GL_LINK_STATUS, &linked);
    if (!linked) {
        std::cerr << "Error: setTexture needs program to be linked." << std::endl;
        return;
    }

    GLint id = glGetUniformLocation(m_progId, name.c_str());
    if (id == -1) {
        cerr << "Fail to set uniform: " << name << endl;
        return;
    }
    switch(count) {
        case 1:
            glUniform1fv(id, 1, val);
            break;
        case 2:
            glUniform2fv(id, 1, val);
            break;
        case 3:
            glUniform3fv(id, 1, val);
            break;
        case 4:
            glUniform4fv(id, 1, val);
            break;
        default:
            break;
    }
}

void GLSLProgram::setTexture(std::string texname, GLint texunit)
{
    GLint linked;
    glGetProgramiv(m_progId, GL_LINK_STATUS, &linked);
    if (!linked) {
        std::cerr << "Error: setTexture needs program to be linked." << std::endl;
        return;
    }

    GLint id = glGetUniformLocation(m_progId, texname.c_str());
    if (id == -1) {
        cerr << "Fail to set texture: " << texname << endl;
        return;
    }

    glUniform1i(id, texunit);
}

void GLSLProgram::addFragmentShader(std::string fragfile)
{
    GLuint object = glCreateShader(GL_FRAGMENT_SHADER);
    if (!object) {
        cerr << "Fail to create shader!" << endl;
        return;
    }

    ifstream stream(fragfile.c_str(), ios::in | ios::binary);
    if (!stream.is_open()) {
        glDeleteShader(object);
        cerr << "Fail to open file: " << fragfile << endl;
        return;
    }

    int start = stream.tellg();
    stream.seekg(0, ios::end);
    int end = stream.tellg();
    stream.seekg(0, ios::beg);

    int length = end - start;
    GLchar *buffer = new GLchar[end - start];
    stream.read(buffer, length);
    stream.close();

    const GLchar *source = buffer;
    glShaderSource(object, 1, &source, &length);
    glCompileShader(object);

    GLint compiled = 0;
    glGetShaderiv(object, GL_COMPILE_STATUS, &compiled);
    if (!compiled) {
        char str[512] = {0};
        glGetShaderInfoLog(object, 512, NULL, str);
        cerr << "Fail to compile source: " << str << endl;
        glDeleteShader(object);
        delete[] buffer;
        return;
    }

    delete[] buffer;

    m_fragShaders.push_back(object);
}

void GLSLProgram::addVertexShader(std::string vertfile)
{
    GLuint object = glCreateShader(GL_VERTEX_SHADER);
    if (!object) {
        cerr << "Fail to create shader!" << endl;
        return;
    }

    ifstream stream(vertfile.c_str(), ios::in | ios::binary);
    if (!stream.is_open()) {
        glDeleteShader(object);
        cerr << "Fail to open file: " << vertfile << endl;
        return;
    }

    int start = stream.tellg();
    stream.seekg(0, ios::end);
    int end = stream.tellg();
    stream.seekg(0, ios::beg);

    int length = end - start;
    GLchar *buffer = new GLchar[end - start];
    stream.read(buffer, length);
    stream.close();

    const GLchar *source = buffer;
    glShaderSource(object, 1, &source, &length);
    glCompileShader(object);

    GLint compiled = 0;
    glGetShaderiv(object, GL_COMPILE_STATUS, &compiled);
    if (!compiled) {
        char str[512] = {0};
        glGetShaderInfoLog(object, 512, NULL, str);
        cerr << "Fail to compile source: " << str << endl;
        glDeleteShader(object);
        delete[] buffer;
        return;
    }

    delete[] buffer;

    m_vertShaders.push_back(object);
}

void GLSLProgram::linkProgram()
{
    m_progId = glCreateProgram();
    if (!m_progId) {
        cerr << "Fail to create program!" << endl;
        return;
    }

    for (vector<GLint>::iterator it=m_vertShaders.begin();it!=m_vertShaders.end();++it)
    {
        glAttachShader(m_progId, *it);
    }

    for (vector<GLint>::iterator it=m_fragShaders.begin();it!=m_fragShaders.end();++it)
    {
        glAttachShader(m_progId, *it);
    }

    glLinkProgram(m_progId);

    GLint linked = 0;
    glGetProgramiv(m_progId, GL_LINK_STATUS, &linked);
    if (!linked) {
        char str[512];
        glGetProgramInfoLog(m_progId, 512, NULL, str);
        cerr << "Fail to link program: " << str << endl;
    }
}
