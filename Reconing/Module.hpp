//
//  Module.hpp
//  Reconing
//
//  Created by apple on 12/04/2021.
//

#ifndef Module_hpp
#define Module_hpp

#include <iostream>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>


class Module {
public:
    Module(std::string name) : name(name) {}
    
    Module() : name("未知") {}
    
    virtual auto update(float delta_time) -> void;
    
    virtual auto update_ui() -> void;
    
    virtual auto destroy() -> bool;
    
    virtual auto render() -> void;
    
    glm::ivec2 window_size;
    std::string name;
    GLFWwindow *window;
};

#endif /* Module_hpp */
