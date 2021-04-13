//
//  Engine.hpp
//  Reconing
//
//  Created by apple on 07/04/2021.
//

#ifndef Engine_hpp
#define Engine_hpp

#include <vector>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <memory>
#include "Module.hpp"

#define ENGINE "引擎"

/// Engine is the class which drives this thing.
class Engine {
public:
    Engine();
    
    auto run() -> int;
    
    auto register_module(Module *module) -> bool;
    
    ~Engine();
    
private:
    GLFWwindow *window;
    glm::ivec2 window_size;
    bool log_autoscroll;
    std::vector<Module *> modules;
    float last_instant;
};

#endif /* Engine_hpp */
