//
//  common.cpp
//  Reconing
//
//  Created by apple on 12/04/2021.
//

#include "common.hpp"
#include <fstream>

std::stringstream log_stream;

auto get_log_stream(std::string type) -> std::ostream & {
    if (log_stream.tellp() != log_stream.beg) {
        log_stream << std::endl;
    }
    log_stream << type << " | ";
    return log_stream;
}

auto get_log() -> const std::stringstream & {
    return log_stream;
}

auto compile(GLuint type, std::string path) -> GLuint {
    std::ifstream reader(path);
    if (!reader.good()) {
        return GL_NONE;
    }
    std::stringstream ss;
    ss << reader.rdbuf();
    auto src = ss.str();
    const auto *buf = src.c_str();
    
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &buf, nullptr);
    glCompileShader(shader);
    
    char log[512] = { 0 };
    glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
    LOG(SHADER) << path << " 编译结果：" << log;
    return shader;
}

auto link(GLuint vertex_shader, GLuint fragment_shader) -> GLuint {
    GLuint program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);
    
    char log[512] = { 0 };
    glGetProgramInfoLog(program, sizeof(log), nullptr, log);
    LOG(SHADER) << "程序链接结果：" << log;
    return program;
}