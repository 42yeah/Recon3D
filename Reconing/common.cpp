//
//  common.cpp
//  Reconing
//
//  Created by apple on 12/04/2021.
//

#include "common.hpp"
#include <fstream>
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

std::stringstream log_stream;

std::mutex _mutex;

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

auto mutex() -> std::mutex & {
    return _mutex;
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


ReconRecord::ReconRecord(std::string name, std::string obj_file) {
    if (name.length() >= 512 || obj_file.length() >= 512) {
        RECON_LOG(RECON_RECORD) << "保存路径过长，会被截掉。";
    }
    name = name.substr(0, std::min((int) name.length(), 511));
    obj_file = obj_file.substr(0, std::min((int) obj_file.length(), 511));

    memcpy(this->name, name.c_str(), name.length());
    memcpy(this->obj_file, obj_file.c_str(), obj_file.length());
}

auto read_recon_records(std::string path) -> std::vector<ReconRecord> {
    std::ifstream reader(path);
    std::vector<ReconRecord> records;
    if (!reader.good()) {
        RECON_LOG(RECON_RECORD) << "无法打开保存列表：" << path;
        return records;
    }
    int count;
    reader.read((char *) &count, sizeof(count));
    for (auto i = 0; i < count; i++) {
        ReconRecord record;
        reader.read((char *) &record, sizeof(record));
        records.push_back(record);
    }
    reader.close();
    return records;
}

auto write_recon_records(const std::vector<ReconRecord> &records, std::string path) -> bool {
    std::ofstream writer(path);
    if (!writer.good()) {
        RECON_LOG(RECON_RECORD) << "无法打开写入重建记录文件：" << path;
        return false;
    }
    auto count = (int) records.size();
    writer.write((char *) &count, sizeof(count));
    for (auto i = 0; i < count; i++) {
        writer.write((char *) &records[i], sizeof(ReconRecord));
    }
    writer.close();
    return true;
}
