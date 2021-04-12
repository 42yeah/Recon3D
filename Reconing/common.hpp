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

auto get_log_stream(std::string type) -> std::ostream &;

auto get_log() -> const std::stringstream &;

#define LOG(MODULE) get_log_stream(MODULE)


#endif /* common_hpp */
