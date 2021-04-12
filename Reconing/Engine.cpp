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
    window = glfwCreateWindow(800, 600, "RECONING", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    gladLoadGL();
    
    // I M G U I ////////////////////////////////////
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    io.IniFilename = nullptr; // Disable ImGui configuration
    ImGui::StyleColorsDark();
    
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330 core");
    io.Fonts->AddFontFromFileTTF("assets/noto_sans_sc.otf", 16, nullptr, io.Fonts->GetGlyphRangesChineseFull());
    log_autoscroll = true;
    
    // O P E N G L //////////////////////////////////
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.1f, 0.2f, 0.3f, 1.0f);
    
    LOG(ENGINE) << "初始化完毕。";
}

auto Engine::run() -> int { 
    while (!glfwWindowShouldClose(window)) {
        // E V E N T S ///////////////////////////////
        glfwPollEvents();
        
        // R E N D E R S /////////////////////////////
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        // I M G U I /////////////////////////////////
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        
        ImGui::SetNextWindowPos({ 10, 10 }, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize({ 300, 200 }, ImGuiCond_FirstUseEver);
        if (ImGui::Begin("记录")) {
            auto str = get_log().str();
            ImGui::Checkbox("自动滚动", &log_autoscroll);
            ImGui::TextWrapped("%s", str.c_str());
            if (log_autoscroll) {
                ImGui::SetScrollHereY(1.0f);
            }
            ImGui::End();
        }
        ImGui::SetNextWindowPos({ 10, 220 }, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize({ 300, 200 }, ImGuiCond_FirstUseEver);
        if (ImGui::Begin("启动向导")) {
            ImGui::Button("选择输入文件夹...");
            ImGui::End();
        }
        ImGui::ShowDemoWindow();
        ImGui::Render();

        glfwGetFramebufferSize(window, &window_size.x, &window_size.y);
        glViewport(0, 0, window_size.x, window_size.y);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }
    return 0;
}

Engine::~Engine() { 
    glfwDestroyWindow(window);
}
