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
#include <stb_image.h>


auto RecordsModule::update_ui() -> void {
    ImGui::SetNextWindowPos({ 320, 10 }, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize({ 200, 200 }, ImGuiCond_FirstUseEver);
    ImGui::Begin("重建历史记录");
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

auto RecordsModule::update(float delta_time) -> bool {
    if (!gl_ready) {
        return false;
    }
    float horizontal_rotation_delta = horizontal_rotation_target - horizontal_rotation;
    horizontal_rotation += (horizontal_rotation_delta * delta_time) * 10.0f;

    model_mat = glm::mat4(1.0f);
    model_mat = glm::rotate(model_mat, glm::radians(180.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    model_mat = glm::rotate(model_mat, glm::radians(horizontal_rotation * 180.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    view_mat = glm::lookAt(glm::vec3(0.0f, 0.0f, radius), center, glm::vec3(0.0f, 1.0f, 0.0f));

    if (glfwGetKey(window, GLFW_KEY_A)) {
        horizontal_rotation_target -= glm::radians(delta_time * 180.0f);
    }
    if (glfwGetKey(window, GLFW_KEY_D)) {
        horizontal_rotation_target += glm::radians(delta_time * 180.0f);
    }
    if (glfwGetKey(window, GLFW_KEY_S)) {
        radius += delta_time * 5.0f;
    }
    if (glfwGetKey(window, GLFW_KEY_W)) {
        radius -= delta_time * 5.0f;
    }
    
    const auto front = glm::normalize(center - eye);
    const auto right = glm::cross(front, glm::vec3(0.0f, 1.0f, 0.0f));
    if (glfwGetKey(window, GLFW_KEY_LEFT)) {
        center -= right * delta_time;
    }
    if (glfwGetKey(window, GLFW_KEY_RIGHT)) {
        center += right * delta_time;
    }
    if (glfwGetKey(window, GLFW_KEY_UP)) {
        center += glm::vec3(0.0f, 1.0f, 0.0f) * delta_time;
    }
    if (glfwGetKey(window, GLFW_KEY_DOWN)) {
        center -= glm::vec3(0.0f, 1.0f, 0.0f) * delta_time;
    }
    return true;
}

auto RecordsModule::load_record() -> bool {
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn, err;
    
    if (!gl_ready) {
        // Is this the first time?
        program = link(compile(GL_VERTEX_SHADER, "shaders/vertex.glsl"),
                       compile(GL_FRAGMENT_SHADER, "shaders/fragment.glsl"));
        eye = glm::vec3(0.0f, 0.0f, 5.0f);
        center = glm::vec3(0.0f);
        perspective_mat = glm::perspective(glm::radians(45.0f), (float) window_size.x / window_size.y, 0.01f, 200.0f);
        view_mat = glm::lookAt(glm::vec3(0.0f, 0.0f, 5.0f), glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    }
    
    auto ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
                                (std::string("recons/") + records[current_selected_index].obj_file).c_str(),
                                "recons/");
    if (!ret) {
        RECON_LOG(RECORDS) << "obj 模型加载失败。警告：" << warn << "，错误：" << err;
        return false;
    }
    RECON_LOG(RECORDS) << "加载完毕。面：" << shapes.size();
    std::vector<Vertex> vertices;
    for (auto i = 0; i < shapes.size(); i++) {
        for (auto f = 0; f < shapes[i].mesh.indices.size() / 3; f++) {
            auto idx0 = shapes[i].mesh.indices[3 * f + 0];
            auto idx1 = shapes[i].mesh.indices[3 * f + 1];
            auto idx2 = shapes[i].mesh.indices[3 * f + 2];
            
            Vertex v0, v1, v2;
            v0 = Vertex {
                { attrib.vertices[3 * idx0.vertex_index + 0],
                    attrib.vertices[3 * idx0.vertex_index + 1] ,
                    attrib.vertices[3 * idx0.vertex_index + 2] },
                { 0.0f, 0.0f, 0.0f }, // There aren't normals
                { attrib.texcoords[2 * idx0.texcoord_index + 0],
                    attrib.texcoords[2 * idx0.texcoord_index + 1] },
                { 1.0f, 0.0f, 0.0f }
            };
            v1 = Vertex {
                { attrib.vertices[3 * idx1.vertex_index + 0],
                    attrib.vertices[3 * idx1.vertex_index + 1],
                    attrib.vertices[3 * idx1.vertex_index + 2] },
                { 0.0f, 0.0f, 0.0f }, // There aren't normals
                { attrib.texcoords[2 * idx1.texcoord_index + 0],
                    attrib.texcoords[2 * idx1.texcoord_index + 1] },
                { 1.0f, 0.0f, 0.0f }
            };
            v2 = Vertex {
                { attrib.vertices[3 * idx2.vertex_index + 0],
                    attrib.vertices[3 * idx2.vertex_index + 1],
                    attrib.vertices[3 * idx2.vertex_index + 2] },
                { 0.0f, 0.0f, 0.0f }, // There aren't normals
                { attrib.texcoords[2 * idx2.texcoord_index + 0],
                    attrib.texcoords[2 * idx2.texcoord_index + 1] },
                { 1.0f, 0.0f, 0.0f }
            };
            vertices.push_back(v0);
            vertices.push_back(v1);
            vertices.push_back(v2);
        }
    }
    glm::vec3 center_of_gravity(0.0f);
    for (const auto &v : vertices) {
        center_of_gravity += v.position;
    }
    center_of_gravity /= vertices.size();
    for (auto &v : vertices) {
        v.position -= center_of_gravity;
        float len = glm::length(v.position);
        if (len > radius) {
            radius = len;
        }
    }
    eye = glm::vec3(0.0f, 0.0f, radius);
    
    if (materials.size() != 1) {
        RECON_LOG(RECORDS) << "材质数量错误。";
        return false;
    }
    
    if (mesh_texture != GL_NONE) {
        glDeleteTextures(1, &mesh_texture);
    }
    glGenTextures(1, &mesh_texture);
    glBindTexture(GL_TEXTURE_2D, mesh_texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    stbi_set_flip_vertically_on_load(true);
    int width, height, channels;
    std::string path = std::string("recons/") + materials[0].diffuse_texname;
    auto *data = stbi_load(path.c_str(), &width, &height, &channels, 0);
    if (!data) {
        RECON_LOG(RECORDS) << "加载材质失败：" << path << " 未找到或无权限";
        return false;
    }
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
    stbi_image_free(data);
    glBindTexture(GL_TEXTURE_2D, GL_NONE);
    
    setup_render(vertices);
    gl_ready = true;
    return true;
}

auto RecordsModule::render() -> void {
    if (!gl_ready) {
        return;
    }
    glUseProgram(program);
    glUniformMatrix4fv(glGetUniformLocation(program, "model"), 1, GL_FALSE, glm::value_ptr(model_mat));
    glUniformMatrix4fv(glGetUniformLocation(program, "view"), 1, GL_FALSE, glm::value_ptr(view_mat));
    glUniformMatrix4fv(glGetUniformLocation(program, "perspective"), 1, GL_FALSE, glm::value_ptr(perspective_mat));
    if (mesh_texture != GL_NONE) {
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, mesh_texture);
        glUniform1i(glGetUniformLocation(program, "tex"), 0);
        glUniform1i(glGetUniformLocation(program, "use_texture"), 1);
    }
    glBindVertexArray(VAO);
    glDrawArrays(GL_TRIANGLES, 0, num_vertices);
    glBindVertexArray(GL_NONE);
}

auto RecordsModule::setup_render(std::vector<Vertex> vertices) -> void {
    if (VAO != GL_NONE) {
        glDeleteVertexArrays(1, &VAO);
    }
    if (VBO != GL_NONE) {
        glDeleteBuffers(1, &VBO);
    }
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * vertices.size(), &vertices[0], GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), nullptr);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const void *) (sizeof(float) * 3));
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const void *) (sizeof(float) * 6));
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const void *) (sizeof(float) * 8));
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glEnableVertexAttribArray(2);
    glEnableVertexAttribArray(3);
    glBindVertexArray(GL_NONE);

    num_vertices = (int) vertices.size();
}


