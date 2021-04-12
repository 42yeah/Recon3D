//
//  ImGuiDemoWindow.cpp
//  Reconing
//
//  Created by apple on 12/04/2021.
//

#include "ImGuiDemoWindow.hpp"
#include <imgui.h>
#include <iostream>


auto ImGuiDemoWindowModule::update_ui() -> void {
    ImGui::ShowDemoWindow();
}
