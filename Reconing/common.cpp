//
//  common.cpp
//  Reconing
//
//  Created by apple on 12/04/2021.
//

#include "common.hpp"

std::stringstream log_stream;

auto get_log_stream(std::string type) -> std::ostream & {
    if (log_stream.tellp() != log_stream.beg) {
        log_stream << std::endl;
    }
    log_stream << type << ": ";
    return log_stream;
}

auto get_log() -> const std::stringstream & {
    return log_stream;
}
