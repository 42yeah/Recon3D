//
//  Engine.hpp
//  Reconing
//
//  Created by apple on 07/04/2021.
//

#ifndef Engine_hpp
#define Engine_hpp

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

/// Engine is the class which drives this thing.
class Engine {
public:
    Engine();
    
    auto run() -> int;
    
    ~Engine();
    
private:
    GLFWwindow *window;
    glm::ivec2 window_size;
    bool log_autoscroll;
};

#endif /* Engine_hpp */
