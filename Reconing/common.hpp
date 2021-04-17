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
#include <vector>
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
    
    ReconRecord(std::string name, std::string obj_file);

    char name[512] = { 0 };
    char obj_file[512] = { 0 };
};

auto read_recon_records(std::string path) -> std::vector<ReconRecord>;

auto write_recon_records(const std::vector<ReconRecord> &records, std::string path) -> bool;


#endif /* common_hpp */
