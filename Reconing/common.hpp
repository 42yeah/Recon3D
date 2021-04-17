//
//  common.hpp
//  Reconing
//
//  Created by apple on 12/04/2021.
//

#ifndef common_hpp
#define common_hpp

#include <iostream>
#include <sstream>
#include <glad/glad.h>

auto get_log_stream(std::string type) -> std::ostream &;

auto get_log() -> const std::stringstream &;

#define RECON_LOG(MODULE) get_log_stream(MODULE)

// S H A D E R S //////////////////////////////////////////
#define SHADER "着色器"
#define RECON_RECORD "重建导出记录"

auto compile(GLuint type, std::string path) -> GLuint;

auto link(GLuint vertex_shader, GLuint fragment_shader) -> GLuint;

// S T O R A G E //////////////////////////////////////////
struct ReconRecord {
    ReconRecord() {}
    
    ReconRecord(std::string path,
                std::string obj_file,
                std::string mtl_file);

    char path[512] = { 0 };
    char obj_file[512] = { 0 };
    char mtl_file[512] = { 0 };
};


#endif /* common_hpp */
