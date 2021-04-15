//
//  Pipeline.cpp
//  Reconing
//
//  Created by apple on 12/04/2021.
//

#include "Pipeline.hpp"
#include <imgui.h>
#include <ImGuiFileDialog.h>
#include <thread>
#include <cstdio>


// P I P E L I N E ///////////////////////////
#define TINYPLY_IMPLEMENTATION
#include <tinyply.h>
#include <GLFW/glfw3.h>

using namespace PipelineNS;

std::mutex mutex;

namespace PipelineNS {

template<typename T>
auto merge(T what) -> T {
    return what;
}

template<typename T, typename ...Ar>
auto merge(T first, Ar... remaining) -> std::string {
    return std::string(first) + " " + merge(remaining...);
}

template<typename ...Ar>
auto invoke(std::function<void(std::string)> callback,
            Ar... cmd) -> bool {
    std::string command = merge(cmd...);
    auto file_ptr = popen(command.c_str(), "r");
    char buf[8192] = { 0 };
    while (fgets(buf, sizeof(buf), file_ptr)) {
        callback(std::string(buf));
    }
    return pclose(file_ptr) == 0;
}


};

auto run_pipeline(Pipeline *pipeline_ptr) -> void {
    auto &pipeline = *pipeline_ptr;
    
    pipeline.run();
}

auto Pipeline::init(std::vector<std::string> image_listing, std::filesystem::path base_path) -> void {
    this->image_listing = image_listing;
    this->base_path = base_path;
    this->state = PipelineState::INTRINSICS_ANALYSIS;
    progress = 0.0f;
}

auto Pipeline::mkdir_if_not_exists(std::filesystem::path path) -> void {
    if (!std::filesystem::exists(path)) {
        std::filesystem::create_directory(path);
    }
}

auto Pipeline::export_to_ply(const std::string path, std::vector<glm::vec3> vertices, std::vector<glm::vec3> camera_poses, std::vector<glm::vec3> colored_points) -> bool {
    mutex.lock();
    RECON_LOG(PIPELINE) << "正在导出模型到 " << path;
    mutex.unlock();
    
    std::ofstream writer(path);
    if (!writer.good()) {
        mutex.lock();
        RECON_LOG(PIPELINE) << "模型导出失败。无法打开文件。";
        mutex.unlock();
        return false;
    }
    writer << "ply" << std::endl
        << "format ascii 1.0" << std::endl
        << "element vertex " << vertices.size() + camera_poses.size() << std::endl
        << "property double x" << std::endl
        << "property double y" << std::endl
        << "property double z" << std::endl
        << "property uchar red" << std::endl
        << "property uchar green" << std::endl
        << "property uchar blue" << std::endl
        << "end_header" << std::endl;

    writer << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1);
    for (auto i = 0; i < vertices.size(); i++) {
        if (i <= colored_points.size() - 1) {
            const auto color = colored_points[i];
            writer << vertices[i].x << ' '
                << vertices[i].y << ' '
                << vertices[i].z << ' '
                << (int) color.x << ' '
                << (int) color.y << ' '
                << (int) color.z << std::endl;
        } else {
            writer << vertices[i].x << ' '
                << vertices[i].y << ' '
                << vertices[i].z << ' '
                << 255 << ' '
                << 255 << ' '
                << 255 << std::endl;
        }
    }
    
    for (auto i = 0; i < camera_poses.size(); i++) {
        writer << camera_poses[i].x << ' '
            << camera_poses[i].y << ' '
            << camera_poses[i].z << ' '
            << 0 << ' '
            << 255 << ' '
            << 0 << std::endl;
    }
    writer.close();
    
    mutex.lock();
    RECON_LOG(PIPELINE) << "模型导出完成。";
    mutex.unlock();
    return true;
}

auto Pipeline::run() -> bool {
    if (std::filesystem::exists("products")) {
        std::filesystem::remove_all("products");
    }
    mkdir_if_not_exists("products");
    mkdir_if_not_exists("products/features");
    mkdir_if_not_exists("products/matches");
    mkdir_if_not_exists("products/sfm");
    mkdir_if_not_exists("products/mvs");
    mkdir_if_not_exists("products/mvs/images");

    if (intrinsics_analysis() &&
        feature_detection() &&
        match_features() &&
        incremental_sfm() &&
        global_sfm() &&
        colorize(PipelineState::COLORIZING) &&
        structure_from_known_poses() &&
        colorize(PipelineState::COLORIZED_ROBUST_TRIANGULATION) &&
        export_openmvg_to_openmvs() &&
        mvs_procedures()) {
        return true;
    }
    mutex.lock();
    RECON_LOG(PIPELINE) << "管线运行错误。";
    mutex.unlock();
    state = PipelineState::FINISHED_ERR;
    return false;
}

auto Pipeline::intrinsics_analysis() -> bool {
    progress = 0.0f;
    state = PipelineState::INTRINSICS_ANALYSIS;
    mutex.lock();
    RECON_LOG(PIPELINE) << "相机内部参数提取开始。";
    mutex.unlock();
    
    // TODO: actually implement intrinsics analysis
    
    mutex.lock();
    RECON_LOG(PIPELINE) << "相机内部参数提取完成。";
    mutex.unlock();
    return true;
}

auto Pipeline::feature_detection() -> bool {
    progress = 0.0f;
    state = PipelineState::FEATURE_DETECTION;
    mutex.lock();
    RECON_LOG(PIPELINE) << "开始特征提取...";
    mutex.unlock();
    
    // TODO: implement feature detection
    
    mutex.lock();
    RECON_LOG(PIPELINE) << "图片特征点提取完成。";
    mutex.unlock();
    return true;
}

auto Pipeline::match_features() -> bool {
    progress = 0.0f;
    state = PipelineState::MATCHING_FEATURES;
    mutex.lock();
    RECON_LOG(PIPELINE) << "开始特征匹配，使用方法：HNSWL2，距离比：0.8，几何模型：基础矩阵。";
    mutex.unlock();

    // TODO: Implement match_features.
    
    mutex.lock();
    progress = 1.0f;
    RECON_LOG(PIPELINE) << "特征匹配结束。";
    mutex.unlock();
    return true;
}

auto Pipeline::incremental_sfm() -> bool {
    progress = 0.0f;
    state = PipelineState::INCREMENTAL_SFM;
    mutex.lock();
    RECON_LOG(PIPELINE) << "开始进行初步 SfM (Structure from Motion)。";
    mutex.unlock();
    
    
    // TODO: Implement incremental SfM.
    
    progress = 1.0f;
    mutex.lock();
    RECON_LOG(PIPELINE) << "初步 SfM 结束。";
    mutex.unlock();
    return true;
}

auto Pipeline::global_sfm() -> bool {
    progress = 0.0f;
    state = PipelineState::GLOBAL_SFM;
    mutex.lock();
    RECON_LOG(PIPELINE) << "开始进行全局 SfM。";
    mutex.unlock();
    
    // TODO: Implement Global SfM.
    
    progress = 1.0f;
    mutex.lock();
    RECON_LOG(PIPELINE) << "全局 SfM 结束。正在更新 SfM 数据...";
    mutex.unlock();
    return true;
}

auto Pipeline::colorize(PipelineState state) -> bool {
    progress = 0.0f;
    this->state = state;
    mutex.lock();
    RECON_LOG(PIPELINE) << "开始进行上色处理。";
    mutex.unlock();
    
    
    // TODO: Implement colorize (at different states).
    
    progress = 1.0f;
    mutex.lock();
    RECON_LOG(PIPELINE) << "上色处理完成。";
    mutex.unlock();
    return true;
}

auto Pipeline::structure_from_known_poses() -> bool {
    state = PipelineState::STRUCTURE_FROM_KNOWN_POSES;
    progress = 0.0f;
    mutex.lock();
    RECON_LOG(PIPELINE) << "正在恢复结构。最大重投影容错：4.0。";
    mutex.unlock();
    
    // TODO: Implement structure from known poses.
    
    progress = 1.0f;
    mutex.lock();
    RECON_LOG(PIPELINE) << "结构恢复完成。";
    mutex.unlock();
    return true;
}

auto Pipeline::export_openmvg_to_openmvs() -> bool {
    progress = 0.0f;
    state = PipelineState::MVG2MVS;
    mutex.lock();
    RECON_LOG(PIPELINE) << "开始转换 OpenMVG 格式 - OpenMVS 格式。";
    mutex.unlock();
    
    // TODO: Implement openMVG - openMVS.
    
    progress = 1.0f;
    mutex.lock();
    RECON_LOG(PIPELINE) << "格式转换完成。";
    mutex.unlock();
    return true;
}

auto Pipeline::mvs_procedures() -> bool {
    // TODO: Implement MVS procedures. Or maybe don't.

    return true;
}



// M O D U L E ///////////////////////////
auto PipelineModule::update_ui() -> void { 
    ImGui::SetNextWindowPos({ 10, 220 }, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize({ 300, 200 }, ImGuiCond_FirstUseEver);
    if (ImGui::Begin("管线向导")) {
        switch (state) {
            case State::ASKING_FOR_INPUT:
                if (ImGui::Button("选择输入文件夹...")) {
                    state = State::CHOOSING_FILE;
                }
                break;
                
            case State::CHOOSING_FILE:
                ImGui::TextWrapped("请选择文件");
                break;
                
            case State::FOLDER_CHOSEN:
                ImGui::TextWrapped("一切已经准备就绪。点击下一步开始。");
                if (ImGui::Button("下一步")) {
                    state = State::RUNNING;
                    std::thread pipeline_runner(run_pipeline, &pipeline);
                    pipeline_runner.detach();
                }
                break;
                
            case State::RUNNING:
                mutex.lock();
                switch (pipeline.state) {
                    case PipelineState::FINISHED_ERR:
                        ImGui::TextWrapped("管线执行出错。检查错误记录获得更多信息。点击 “重试” 重新执行向导。");
                        if (ImGui::Button("重试")) {
                            state = State::ASKING_FOR_INPUT;
                        }
                        break;
                        
                    case PipelineState::FINISHED_SUCCESS:
                        ImGui::TextWrapped("管线执行完毕。点击 “重试” 重新执行向导。");
                        if (ImGui::Button("重试")) {
                            state = State::ASKING_FOR_INPUT;
                        }
                        break;
                        
                    case PipelineState::INTRINSICS_ANALYSIS:
                        ImGui::TextWrapped("正在检视相机内部参数...");
                        break;
                        
                    case PipelineState::FEATURE_DETECTION:
                        ImGui::TextWrapped("正在进行特征提取...");
                        break;
                        
                    case PipelineState::MATCHING_FEATURES:
                        ImGui::TextWrapped("正在两两匹配特征点...");
                        break;
                        
                    case PipelineState::INCREMENTAL_SFM:
                        ImGui::TextWrapped("正在进行初步 SfM 处理...");
                        break;
                        
                    case PipelineState::GLOBAL_SFM:
                        ImGui::TextWrapped("正在进行全局 SfM 处理...");
                        break;
                        
                    case PipelineState::COLORIZING:
                        ImGui::TextWrapped("正在对模型进行上色...");
                        break;
                        
                    case PipelineState::STRUCTURE_FROM_KNOWN_POSES:
                        ImGui::TextWrapped("正在从已知相机坐标构建结构...");
                        break;
                        
                    case PipelineState::COLORIZED_ROBUST_TRIANGULATION:
                        ImGui::TextWrapped("正在对鲁棒模型进行上色...");
                        break;
                        
                    case PipelineState::MVG2MVS:
                        ImGui::TextWrapped("正在从 OpenMVG 格式转换到 OpenMVS 格式...");
                        break;
                        
                    case PipelineState::DENSIFY_PC:
                        ImGui::TextWrapped("正在稠密化点云...");
                        break;
                        
                    case PipelineState::RECONSTRUCT_MESH:
                        ImGui::TextWrapped("正在重建网格模型...");
                        break;
                }
                mutex.unlock();
                if (pipeline.state != PipelineState::FINISHED_ERR &&
                    pipeline.state != PipelineState::FINISHED_SUCCESS) {
                    ImGui::ProgressBar(pipeline.progress);
                }
                break;
        }
        ImGui::End();
    }
    if (state == State::CHOOSING_FILE) {
        ImGuiFileDialog::Instance()->OpenDialog("Folder", "选择输入目录...", nullptr, ".");
        ImGui::SetNextWindowSize({ 500, 300 }, ImGuiCond_FirstUseEver);
        if (ImGuiFileDialog::Instance()->Display("Folder")) {
            if (ImGuiFileDialog::Instance()->IsOk()) {
                std::string path = ImGuiFileDialog::Instance()->GetCurrentPath();
                RECON_LOG(PIPELINE) << "目录已选定：" << path;
                RECON_LOG(PIPELINE) << "有效数据：" <<
                    list_images(path);
                state = State::FOLDER_CHOSEN;
                pipeline.init(image_listing, path);
            }
            ImGuiFileDialog::Instance()->Close();
        }
    }
}

auto PipelineModule::update(float delta_time) -> void {
    if (!opengl_ready && pipeline.state == PipelineState::GLOBAL_SFM) {
        program = link(compile(GL_VERTEX_SHADER, "shaders/vertex.glsl"),
                       compile(GL_FRAGMENT_SHADER, "shaders/fragment.glsl"));

        load_ply_as_pointcloud("products/sfm/clouds_and_poses.ply");
        
        eye = glm::vec3(0.0f, 0.0f, 5.0f);
        center = glm::vec3(0.0f, 0.0f, 0.0f);
        perspective_mat = glm::perspective(glm::radians(45.0f), (float) window_size.x / window_size.y, 0.01f, 200.0f);
        
        opengl_ready = true;
        time = 0.0f;
    }
    time += delta_time;
    
    if (glfwGetKey(window, GLFW_KEY_W)) {
        radius += delta_time * 5.0f;
    }
    if (glfwGetKey(window, GLFW_KEY_S)) {
        radius -= delta_time * 5.0f;
    }
    
    eye = glm::vec3(sinf(time) * radius, 0.0f, cosf(time) * radius);
    view_mat = glm::lookAt(eye, center, glm::vec3(0.0f, 1.0f, 0.0f));

    
}

auto PipelineModule::list_images(std::filesystem::path path) -> int {
    auto legit_image = 0;
    image_listing.clear();
    for (const auto &entry : std::filesystem::directory_iterator(path)) {
        const auto fext = entry.path().extension();
        if (fext != ".jpg" && fext != ".bmp" && fext != ".png" && fext != ".jpeg") {
            continue;
        }
        image_listing.push_back(entry.path().string());
        legit_image++;
    }
    return legit_image;
}

auto PipelineModule::render() -> void { 
    if (!opengl_ready) {
        // Not ready yet
        return;
    }
    glUseProgram(program);
    glUniformMatrix4fv(glGetUniformLocation(program, "view"), 1, GL_FALSE, glm::value_ptr(view_mat));
    glUniformMatrix4fv(glGetUniformLocation(program, "perspective"), 1, GL_FALSE, glm::value_ptr(perspective_mat));
    glBindVertexArray(VAO);
    glPointSize(5.0f);
    glDrawArrays(GL_POINTS, 0, num_vertices);
    glBindVertexArray(GL_NONE);
}

auto PipelineModule::load_ply_as_pointcloud(std::string path) -> void {
    tinyply::PlyFile file;
    std::ifstream reader(path);
    if (!reader.good()) {
        RECON_LOG(PIPELINE) << "无法读取 ply 路径：" << path;
        return;
    }
    file.parse_header(reader);
    RECON_LOG(PIPELINE) << path << " 头部解析成功：是 " << (file.is_binary_file() ? "二进制" : "纯文本");
    
    std::shared_ptr<tinyply::PlyData> vertices, normals, colors, tex_coords, faces, tripstrip;
    try { vertices = file.request_properties_from_element("vertex", { "x", "y", "z" }); }
    catch (const std::exception & e) { RECON_LOG(PIPELINE) << "tinyply 错误: " << e.what() << std::endl; }
    
    file.read(reader);
    reader.close();

    RECON_LOG(PIPELINE) << "读取完毕。节点数量：" << vertices->count;
    const auto num_bytes = vertices->buffer.size_bytes();
    std::vector<glm::vec3> verts(vertices->count);
    
    if (vertices->t == tinyply::Type::FLOAT64) {
        std::vector<glm::dvec3> verts64(vertices->count);
        std::memcpy(verts64.data(), vertices->buffer.get(), num_bytes);

        for (auto i = 0; i < vertices->count; i++) {
            verts[i] = { verts64[i].x,
                verts64[i].y,
                verts64[i].z };
        }
    } else {
        std::memcpy(verts.data(), vertices->buffer.get(), num_bytes);
    }

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

    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * verts.size(), &verts[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 3, nullptr);
    glBindVertexArray(GL_NONE);
    
    num_vertices = (int) verts.size();
}

