//
//  Engine.cpp
//  Reconing
//
//  Created by apple on 07/04/2021.
//

#include "Engine.hpp"
#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include "common.hpp"


Engine::Engine() { 
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    window = glfwCreateWindow(1200, 800, "RECONING", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    gladLoadGL();
    
    // I M G U I ////////////////////////////////////
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    io.IniFilename = nullptr; // Disable ImGui configuration
    ImGui::StyleColorsDark();
    
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330 core");
    io.Fonts->AddFontFromFileTTF("assets/noto_sans_sc.otf", 18, nullptr, io.Fonts->GetGlyphRangesChineseFull());
    log_autoscroll = true;
    
    ImGuiStyle &style = ImGui::GetStyle();
    style.FrameRounding = 7.0f;
    style.WindowRounding = 7.0f;
    
    // O P E N G L //////////////////////////////////
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    
    // C H R O N O L O G Y //////////////////////////
    last_instant = glfwGetTime();
    
    // R E N D E R S ////////////////////////////////
    currently_selected_render = 0;
    available_renders.clear();
    
    RECON_LOG(ENGINE) << "初始化完毕。";
}

auto Engine::run() -> int { 
    while (!glfwWindowShouldClose(window)) {
        // C H R O N O L O G Y ///////////////////////
        float this_instant = glfwGetTime();
        auto delta_time = this_instant - last_instant;
        last_instant = this_instant;
        
        // E V E N T S ///////////////////////////////
        glfwPollEvents();
        glfwGetFramebufferSize(window, &window_size.x, &window_size.y);
        auto old_render_size = available_renders.size();
        available_renders.clear();
        for (auto i = 0; i < modules.size(); i++) {
            auto &m = modules[i];
            m->window_size = { window_size.x, window_size.y };
            m->window = window;
            if (m->update(delta_time)) {
                available_renders.push_back(i);
            }
        }
        if (available_renders.size() != old_render_size) {
            currently_selected_render = 0;
        }

        // R E N D E R S /////////////////////////////
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if (available_renders.size() > currently_selected_render) {
            modules[available_renders[currently_selected_render]]->render();
        }

        // I M G U I /////////////////////////////////
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        
        ImGui::SetNextWindowPos({ 10, 10 }, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize({ 300, 200 }, ImGuiCond_FirstUseEver);
        ImGui::Begin("日志");
        auto str = get_log().str();
        ImGui::TextWrapped("%s", str.c_str());
        ImGui::Checkbox("自动滚动", &log_autoscroll);
        if (log_autoscroll) {
            ImGui::SetScrollHereY(1.0f);
        }
        ImGui::End();
        for (auto &m : modules) {
            m->update_ui();
        }
        if (available_renders.size() > 1) {
            ImGui::SetNextWindowPos({ 50, 50 }, ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSize({ 300, 200 }, ImGuiCond_FirstUseEver);
            ImGui::Begin("选择输出 ");
            ImGui::TextWrapped("大于一个模块尝试渲染内容到主界面。选择一个你想要查看的模块：");
            if (ImGui::BeginListBox("", ImVec2 { -FLT_MIN, 5 * ImGui::GetTextLineHeightWithSpacing() })) {
                for (auto i = 0; i < available_renders.size(); i++) {
                    auto selected = currently_selected_render == i;
                    if (ImGui::Selectable(modules[available_renders[i]]->name.c_str(),
                                          selected)) {
                        currently_selected_render = i;
                    }
                    if (selected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndListBox();
            }
            ImGui::End();
        }
        ImGui::Render();

        glViewport(0, 0, window_size.x, window_size.y);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }
    return 0;
}

Engine::~Engine() {
    for (auto &m : modules) {
        m->destroy();
    }
    glfwDestroyWindow(window);
}

auto Engine::register_module(Module *module) -> bool {
    modules.push_back(module);
    RECON_LOG(ENGINE) << "模块已注册：" << module->name;
    return true;
}
