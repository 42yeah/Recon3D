//
//  Records.cpp
//  Reconing
//
//  Created by apple on 17/04/2021.
//

#include "Records.hpp"
#include <imgui.h>
#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>


auto RecordsModule::update_ui() -> void {
    ImGui::SetNextWindowPos({ 320, 10 }, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize({ 200, 200 }, ImGuiCond_FirstUseEver);
    if (ImGui::Begin("重建历史记录")) {
        if (ImGui::Button("刷新")) {
            records = read_recon_records("recons/records.bin");
        }
        if (records.size() > 0) {
            ImGui::SameLine();
            if (ImGui::Button("加载")) {
                // Load module and display (render)
                if (!load_record()) {
                    RECON_LOG(RECORDS) << "加载重建记录失败。";
                }
            }
        }
        if (records.size() > 0 &&
            ImGui::BeginListBox("recon list", ImVec2 { -FLT_MIN, 5 * ImGui::GetTextLineHeightWithSpacing() })) {
            for (auto i = 0; i < records.size(); i++) {
                const auto is_selected = current_selected_index == i;
                if (ImGui::Selectable(records[i].name, is_selected)) {
                    current_selected_index = i;
                }
                if (is_selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndListBox();
        } else if (records.size() <= 0) {
            ImGui::TextWrapped("现在还没有重建记录。");
        }
        ImGui::End();
    }
}

auto RecordsModule::update(float delta_time) -> bool { 
    return false;
}

auto RecordsModule::load_record() -> bool {
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn, err;
    
    auto ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, records[current_selected_index].obj_file);
    if (!ret) {
        RECON_LOG(RECORDS) << "obj 模型加载失败。警告：" << warn << "，错误：" << err;
        return false;
    }
    RECON_LOG(RECORDS) << "加载完毕。面：" << shapes.size();
    for (auto i = 0; i < shapes.size(); i++) {
        auto index_offset = 0;
        for (auto face_id = 0; face_id < shapes[i].mesh.num_face_vertices.size(); face_id++) {
            auto num_face_vertices = shapes[i].mesh.num_face_vertices[face_id];
            for (auto i = 0; i < num_face_vertices; i++) {
                
            }
        }
    }
    return true;
}
