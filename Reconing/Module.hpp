//
//  Module.hpp
//  Reconing
//
//  Created by apple on 12/04/2021.
//

#ifndef Module_hpp
#define Module_hpp

#include <iostream>


class Module {
public:
    Module(std::string name) : name(name) {}
    
    Module() : name("未知") {}
    
    virtual auto update() -> void;
    
    virtual auto update_ui() -> void;
    
    virtual auto destroy() -> bool;
    
    std::string name;
};

#endif /* Module_hpp */
