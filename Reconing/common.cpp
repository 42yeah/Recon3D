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
    RECON_LOG(SHADER) << path << " 编译结果：" << log;
    return shader;
}

auto link(GLuint vertex_shader, GLuint fragment_shader) -> GLuint {
    GLuint program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);
    
    char log[512] = { 0 };
    glGetProgramInfoLog(program, sizeof(log), nullptr, log);
    RECON_LOG(SHADER) << "程序链接结果：" << log;
    return program;
}


ReconRecord::ReconRecord(std::string path, std::string obj_file, std::string mtl_file) { 
    if (path.length() >= 512 || obj_file.length() >= 512 || mtl_file.length() >= 512) {
        RECON_LOG(RECON_RECORD) << "保存路径过长，会被截掉。";
    }
    path = path.substr(511);
    obj_file = obj_file.substr(511);
    mtl_file = mtl_file.substr(511);
    
    memcpy(this->path, path.c_str(), path.length());
    memcpy(this->obj_file, obj_file.c_str(), obj_file.length());
    memcpy(this->mtl_file, mtl_file.c_str(), mtl_file.length());
}
