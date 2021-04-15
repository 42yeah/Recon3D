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

auto compile(GLuint type, std::string path) -> GLuint;

auto link(GLuint vertex_shader, GLuint fragment_shader) -> GLuint;


#endif /* common_hpp */
