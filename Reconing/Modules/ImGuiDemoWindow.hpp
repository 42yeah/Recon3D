//
//  ImGuiDemoWindow.hpp
//  Reconing
//
//  Created by apple on 12/04/2021.
//

#ifndef ImGuiDemoWindow_hpp
#define ImGuiDemoWindow_hpp

#include "Module.hpp"

class ImGuiDemoWindowModule : public Module {
public:
    ImGuiDemoWindowModule() : Module("ImGui Demo 模块") {}

    virtual auto update_ui() -> void override;
};

#endif /* ImGuiDemoWindow_hpp */
