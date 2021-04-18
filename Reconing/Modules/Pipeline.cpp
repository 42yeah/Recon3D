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
#include <stb_image.h>

using namespace PipelineNS;

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

auto point_fetch(std::shared_ptr<tinyply::PlyData> ply_data, int offset) -> glm::vec3 {
    glm::vec3 result(0.0f);
    
    const auto *buf = ply_data->buffer.get();
    switch (ply_data->t) {
        case tinyply::Type::FLOAT32:
            result = { ((glm::vec3 *) (buf))[offset].x,
                ((glm::vec3 *) (buf))[offset].y,
                ((glm::vec3 *) (buf))[offset].z };
            break;
            
        case tinyply::Type::FLOAT64:
            result = { (float) ((glm::dvec3 *) (buf))[offset].x,
                (float) ((glm::dvec3 *) (buf))[offset].y,
                (float) ((glm::dvec3 *) (buf))[offset].z };
            break;
            
        case tinyply::Type::INT8:
            result = { (float) ((glm::i8vec3 *) (buf))[offset].x / 255,
                (float) ((glm::i8vec3 *) (buf))[offset].y / 255,
                (float) ((glm::i8vec3 *) (buf))[offset].z / 255 };
            break;
            
        case tinyply::Type::UINT8:
            result = { (float) ((glm::u8vec3 *) (buf))[offset].x / 255,
                (float) ((glm::u8vec3 *) (buf))[offset].y / 255,
                (float) ((glm::u8vec3 *) (buf))[offset].z / 255 };
            break;
            
        default:
            // Unsupported, because I don't want to
            break;
    }
    return result;
}



};

auto run_pipeline(Pipeline *pipeline_ptr) -> void {
    auto &pipeline = *pipeline_ptr;
    
    pipeline.run();
}

auto Pipeline::init(std::vector<std::string> image_listing, std::filesystem::path base_path,
                    std::string mvg_executable_path,
                    std::string mvs_executable_path) -> void {
    this->image_listing = image_listing;
    this->base_path = base_path;
    this->state = PipelineState::INTRINSICS_ANALYSIS;
    this->mvg_executable_path = mvg_executable_path;
    this->mvs_executable_path = mvs_executable_path;
    progress = 0.0f;
}

auto Pipeline::mvs() -> std::filesystem::path {
    return mvs_executable_path;
}

auto Pipeline::mvg() -> std::filesystem::path {
    return mvg_executable_path;
}

auto Pipeline::export_to_ply(const std::string path, std::vector<glm::vec3> vertices, std::vector<glm::vec3> camera_poses, std::vector<glm::vec3> colored_points) -> bool {
    mutex().lock();
    RECON_LOG(PIPELINE) << "正在导出模型到 " << path;
    mutex().unlock();
    
    std::ofstream writer(path);
    if (!writer.good()) {
        mutex().lock();
        RECON_LOG(PIPELINE) << "模型导出失败。无法打开文件。";
        mutex().unlock();
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
    
    mutex().lock();
    RECON_LOG(PIPELINE) << "模型导出完成。";
    mutex().unlock();
    return true;
}

auto Pipeline::run() -> bool {
    cleanup();
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
        density_pointcloud() &&
        reconstruct_mesh() &&
        refine_mesh() &&
        texture_mesh()) {
        state = PipelineState::FINISHED_SUCCESS;
        return true;
    }
    mutex().lock();
    RECON_LOG(PIPELINE) << "管线运行错误。";
    mutex().unlock();
    state = PipelineState::FINISHED_ERR;
    return false;
}

auto Pipeline::intrinsics_analysis() -> bool {
    state = PipelineState::INTRINSICS_ANALYSIS;
    progress = (float) state / (float) PipelineState::NUM_PROCEDURES;
    mutex().lock();
    RECON_LOG(PIPELINE) << "相机内部参数提取开始。";
    mutex().unlock();

    auto ret = invoke([&] (std::string message) {
    }, mvg() / "openMVG_main_SfMInit_ImageListing", "-i", base_path, "-o", "products/matches", "-f", "2500.0");

    mutex().lock();
    RECON_LOG(PIPELINE) << "相机内部参数提取完成。";
    mutex().unlock();
    return ret == 1;
}

auto Pipeline::feature_detection() -> bool {
    state = PipelineState::FEATURE_DETECTION;
    progress = (float) state / (float) PipelineState::NUM_PROCEDURES;
    mutex().lock();
    RECON_LOG(PIPELINE) << "开始特征提取...";
    mutex().unlock();
    
    auto ret = invoke([&] (std::string message) {
    }, mvg() / "openMVG_main_ComputeFeatures",
           "-i" "products/matches/sfm_data.json",
           "-o", "products/matches/",
           "-m", "SIFT",
           "-n", "4");
    
    mutex().lock();
    RECON_LOG(PIPELINE) << "图片特征点提取完成。";
    mutex().unlock();
    return ret == 1;
}

auto Pipeline::match_features() -> bool {
    state = PipelineState::MATCHING_FEATURES;
    progress = (float) state / (float) PipelineState::NUM_PROCEDURES;
    mutex().lock();
    RECON_LOG(PIPELINE) << "开始特征匹配，使用方法：HNSWL2，距离比：0.8，几何模型：基础矩阵。";
    mutex().unlock();

    auto ret = invoke([&] (std::string message) {
    }, mvg() / "openMVG_main_ComputeMatches",
           "-i", "products/matches/sfm_data.json",
           "-o", "products/matches/",
           "-n", "HNSWL2",
           "-r", "0.8");
    
    mutex().lock();
    RECON_LOG(PIPELINE) << "特征匹配结束。";
    mutex().unlock();
    return ret == 1;
}

auto Pipeline::incremental_sfm() -> bool {
    state = PipelineState::INCREMENTAL_SFM;
    progress = (float) state / (float) PipelineState::NUM_PROCEDURES;
    mutex().lock();
    RECON_LOG(PIPELINE) << "开始进行初步 SfM (Structure from Motion)。";
    mutex().unlock();
    
    auto ret = invoke([&] (std::string message) {
    }, mvg() / "openMVG_main_IncrementalSfM",
           "-i", "products/matches/sfm_data.json",
           "-m", "products/matches/",
           "-o", "products/sfm/");
    
    mutex().lock();
    RECON_LOG(PIPELINE) << "初步 SfM 结束。";
    mutex().unlock();
    return ret == 1;
}

auto Pipeline::global_sfm() -> bool {
    state = PipelineState::GLOBAL_SFM;
    progress = (float) state / (float) PipelineState::NUM_PROCEDURES;
    mutex().lock();
    RECON_LOG(PIPELINE) << "开始进行全局 SfM。";
    mutex().unlock();
    
    auto ret = invoke([&] (std::string message) {
    }, mvg() / "openMVG_main_GlobalSfM",
           "-i", "products/matches/sfm_data.json",
           "-M", "products/matches/matches.f.bin",
           "-m", "products/matches/",
           "-o", "products/sfm/");
    
    mutex().lock();
    RECON_LOG(PIPELINE) << "全局 SfM 结束。正在更新 SfM 数据...";
    mutex().unlock();
    return ret == 1;
}

auto Pipeline::colorize(PipelineState state) -> bool {
    this->state = state;
    progress = (float) state / (float) PipelineState::NUM_PROCEDURES;
    mutex().lock();
    RECON_LOG(PIPELINE) << "开始进行上色处理。";
    mutex().unlock();
    
    auto ret = 0;
    if (state == PipelineState::COLORIZING) {
        ret = invoke([&] (std::string message) {
        }, mvg() / "openMVG_main_ComputeSfM_DataColor",
               "-i", "products/sfm/sfm_data.bin",
               "-o", "products/sfm/colorized.ply");
    } else if (state == PipelineState::COLORIZED_ROBUST_TRIANGULATION) {
        ret = invoke([&] (std::string message) {
        }, mvg() / "openMVG_main_ComputeSfM_DataColor",
               "-i", "products/sfm/robust.bin",
               "-o", "products/sfm/robust_colorized.ply");
    } else {
        mutex().lock();
        RECON_LOG(PIPELINE) << "错误！未知上色阶段。";
        mutex().unlock();
        return false;
    }
    
    mutex().lock();
    RECON_LOG(PIPELINE) << "上色处理完成。";
    mutex().unlock();
    return ret == 1;
}

auto Pipeline::structure_from_known_poses() -> bool {
    state = PipelineState::STRUCTURE_FROM_KNOWN_POSES;
    progress = (float) state / (float) PipelineState::NUM_PROCEDURES;
    mutex().lock();
    RECON_LOG(PIPELINE) << "正在恢复结构。最大重投影容错：4.0。";
    mutex().unlock();
    
    auto ret = invoke([&] (std::string message) {
    }, mvg() / "openMVG_main_ComputeStructureFromKnownPoses",
           "-i", "products/sfm/sfm_data.bin",
           "-m", "products/matches/",
           "-r", "4.0",
           "-f", "products/matches/matches.f.bin",
           "-o", "products/sfm/robust.bin");

    mutex().lock();
    RECON_LOG(PIPELINE) << "结构恢复完成。";
    mutex().unlock();
    return ret == 1;
}

auto Pipeline::export_openmvg_to_openmvs() -> bool {
    state = PipelineState::MVG2MVS;
    progress = (float) state / (float) PipelineState::NUM_PROCEDURES;
    mutex().lock();
    RECON_LOG(PIPELINE) << "开始转换 OpenMVG 格式 - OpenMVS 格式。";
    mutex().unlock();
    
    auto ret = invoke([&] (std::string) {
    }, mvg() / "openMVG_main_openMVG2openMVS",
           "-i", "products/sfm/sfm_data.bin",
           "-o", "products/mvs/scene.mvs",
           "-d", "products/mvs/images");

    mutex().lock();
    RECON_LOG(PIPELINE) << "格式转换完成。";
    mutex().unlock();
    return ret == 1;
}

auto Pipeline::density_pointcloud() -> bool {
    state = PipelineState::DENSIFY_PC;
    progress = (float) state / (float) PipelineState::NUM_PROCEDURES;
    mutex().lock();
    RECON_LOG(PIPELINE) << "开始稠密化点云。";
    mutex().unlock();
    
    auto ret = invoke([&] (std::string) {
    }, mvs() / "DensifyPointCloud", "products/mvs/scene.mvs",
           "--dense-config-file", "densify.ini",
           "--resolution-level", "1",
           "-w", ".");
    
    mutex().lock();
    RECON_LOG(PIPELINE) << "稠密化点云完成。";
    mutex().unlock();
    return ret == 1;
}

auto Pipeline::reconstruct_mesh() -> bool { 
    state = PipelineState::RECONSTRUCT_MESH;
    progress = (float) state / (float) PipelineState::NUM_PROCEDURES;
    mutex().lock();
    RECON_LOG(PIPELINE) << "开始重建网格。";
    mutex().unlock();
    
    auto ret = invoke([&] (std::string) {
    }, mvs() / "ReconstructMesh", "products/mvs/scene_dense.mvs",
           "-w", ".");
    
    mutex().lock();
    RECON_LOG(PIPELINE) << "重建网格完成。";
    mutex().unlock();
    return ret == 1;
}

auto Pipeline::refine_mesh() -> bool {
    state = PipelineState::REFINE_MESH;
    progress = (float) state / (float) PipelineState::NUM_PROCEDURES;
    mutex().lock();
    RECON_LOG(PIPELINE) << "开始修正网格。迭代数：2";
    mutex().unlock();
    
    auto ret = invoke([&] (std::string) {
    }, mvs() / "RefineMesh", "products/mvs/scene_dense_mesh.mvs",
                      "--scales", "2",
                      "-w", ".");

    mutex().lock();
    RECON_LOG(PIPELINE) << "网格修正完成。";
    mutex().unlock();
    return ret == 1;
}

auto Pipeline::texture_mesh() -> bool {
    state = PipelineState::TEXTURE_MESH;
    progress = (float) state / (float) PipelineState::NUM_PROCEDURES;
    mutex().lock();
    RECON_LOG(PIPELINE) << "开始网格贴图。";
    mutex().unlock();
    
    auto ret = invoke([&] (std::string) {
    }, mvs() / "TextureMesh", "products/mvs/scene_dense_mesh_refine.mvs",
                      "--decimate", "0.5",
                      "-w", ".");
    
    mutex().lock();
    RECON_LOG(PIPELINE) << "网格贴图完成。";
    mutex().unlock();
    return ret == 1;
}

auto Pipeline::save_session(std::string name) -> bool {
    tinyply::PlyFile file;
    std::string path = "products/mvs/scene_dense_mesh_refine_texture.ply";
    std::ifstream reader(path);
    if (!reader.good()) {
        RECON_LOG(PIPELINE) << "无法读取 ply 路径：" << path;
        return false;
    }
    file.parse_header(reader);
    RECON_LOG(PIPELINE) << path << " 头部解析成功：是 " << (file.is_binary_file() ? "二进制" : "纯文本");
    
    std::shared_ptr<tinyply::PlyData> vertices, normals, colors, tex_coords, faces, tripstrip;
    try { vertices = file.request_properties_from_element("vertex", { "x", "y", "z" }); }
    catch (const std::exception & e) { RECON_LOG(PIPELINE) << "tinyply 错误: " << e.what(); }

    try { faces = file.request_properties_from_element("face", { "vertex_indices" }, 3); }
    catch (const std::exception & e) { RECON_LOG(PIPELINE) << "tinyply 无法读取面: " << e.what(); }
    
    try { tex_coords = file.request_properties_from_element("face", { "texcoord" }, 6); }
    catch (const std::exception & e) { RECON_LOG(PIPELINE) << "tinyply 错误: " << e.what(); }
    
    file.read(reader);
    reader.close();
    
    RECON_LOG(PIPELINE) << "ply 读取结束。开始转存为 .obj...";
    mkdir_if_not_exists("recons");
    std::ofstream obj_writer("recons/" + name + ".obj");
    std::ofstream mtl_writer("recons/" + name + ".mtl");
    if (!obj_writer.good() || !mtl_writer.good()) {
        if (obj_writer.good()) {
            obj_writer.close();
        }
        if (mtl_writer.good()) {
            mtl_writer.close();
        }
        RECON_LOG(PIPELINE) << "错误！无法打开 obj 进行写入。";
        return false;
    }
    obj_writer << "mtllib " << name << ".mtl" << std::endl << std::endl;
    for (auto i = 0; i < vertices->count; i++) {
        glm::vec3 vertex = point_fetch(vertices, i);
        obj_writer << "v " << vertex.x << " " << vertex.y << " " << vertex.z << std::endl;
    }
    float *uv_arr = (float *) tex_coords->buffer.get();
    for (auto i = 0; i < faces->count; i++) {
        // For each face, there are 3 texture coordinates
        obj_writer << "vt " << uv_arr[i * 6 + 0] << " " << uv_arr[i * 6 + 1] << std::endl;
        obj_writer << "vt " << uv_arr[i * 6 + 2] << " " << uv_arr[i * 6 + 3] << std::endl;
        obj_writer << "vt " << uv_arr[i * 6 + 4] << " " << uv_arr[i * 6 + 5] << std::endl;
    }
    
    glm::u32vec3 *indices = (glm::u32vec3 *) faces->buffer.get();
    obj_writer << "g face" << std::endl;
    for (auto i = 0; i < faces->count; i++) {
        const auto &index = indices[i];
        obj_writer << "f " << (index.x + 1) << "/" << ((i * 3) + 1) << " " << (index.y + 1) << "/" << ((i * 3) + 2)
            << " " << (index.z + 1) << "/" << ((i * 3) + 3) << std::endl;
    }
    obj_writer.close();
    
    std::string texture_name = name + ".png";
    std::filesystem::copy("products/mvs/scene_dense_mesh_refine_texture.png", std::string("recons/") + texture_name);
    mtl_writer << "newmtl Material" << std::endl;
    mtl_writer << "map_Ka " << texture_name << std::endl;
    mtl_writer << "map_Kd " << texture_name << std::endl;
    mtl_writer.close();
    
    ReconRecord record(name, name + ".obj");
    auto records = read_recon_records("recons/records.bin");
    records.push_back(record);
    write_recon_records(records, "recons/records.bin");

    return true;
}

auto Pipeline::cleanup() -> void { 
    rm_if_exists("densify.ini");
    for (auto i = 0; i < image_listing.size(); i++) {
        char name_raw[512] = { 0 };
        sprintf(name_raw, "depth%04d", i);
        std::string name(name_raw);
        rm_if_exists(name + ".png");
        rm_if_exists(name + ".conf.png");
        rm_if_exists(name + ".dmap");
        rm_if_exists(name + ".filtered.ply");
        rm_if_exists(name + ".filtered.png");
        rm_if_exists(name + ".ply");
    }
    rm_if_exists("MeshRefine0.ply");
    rm_if_exists("MeshRefine1.ply");
    rm_if_exists("MeshRefined1.ply");
    for (auto &entry : std::filesystem::directory_iterator(".")) {
        if (entry.path().extension() == ".log") {
            rm_if_exists(entry.path());
        }
    }
}

auto Pipeline::rm_if_exists(std::filesystem::path path) -> void { 
    if (std::filesystem::exists(path)) {
        std::filesystem::remove(path);
    }
}



// M O D U L E ///////////////////////////
auto PipelineModule::update_ui() -> void { 
    ImGui::SetNextWindowPos({ 10, 220 }, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize({ 300, 200 }, ImGuiCond_FirstUseEver);
    ImGui::Begin("管线向导");
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
                mesh_texture = GL_NONE; // Reset mesh_texture so we won't accidentally sample it
                std::thread pipeline_runner(run_pipeline, &pipeline);
                pipeline_runner.detach();
            }
            break;
            
        case State::RUNNING:
            mutex().lock();
            switch (pipeline.state) {
                case PipelineState::FINISHED_ERR:
                    ImGui::TextWrapped("管线执行出错。检查错误记录获得更多信息。点击 “重试” 重新执行向导。");
                    if (ImGui::Button("重试")) {
                        state = State::ASKING_FOR_INPUT;
                    }
                    break;
                    
                case PipelineState::FINISHED_SUCCESS:
                    ImGui::TextWrapped("管线执行完毕。点击 “重试” 重新执行向导。点击 “保存” 保存到历史中。");
                    ImGui::InputText("保存名称", session_name, sizeof(session_name));
                    if (ImGui::Button("保存")) {
                        RECON_LOG(PIPELINE) << "正在保存重建记录...";
                        if (!pipeline.save_session(std::string(session_name))) {
                            RECON_LOG(PIPELINE) << "记录保存失败。";
                        } else {
                            RECON_LOG(PIPELINE) << "记录保存完成。";
                        }
                    }
                    ImGui::SameLine();
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
                    
                case PipelineState::REFINE_MESH:
                    ImGui::TextWrapped("正在修正网格...");
                    break;
                    
                case PipelineState::TEXTURE_MESH:
                    ImGui::TextWrapped("正在对网格进行贴图...");
                    break;
                    
                default:
                    break;
            }
            mutex().unlock();
            if (pipeline.state != PipelineState::FINISHED_ERR &&
                pipeline.state != PipelineState::FINISHED_SUCCESS) {
                ImGui::ProgressBar(pipeline.progress);
            }
            break;
    }
    ImGui::End();
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
                pipeline.init(image_listing,
                              path,
                              "/Users/apple/Projects/openMVG/build/Darwin-x86_64-DEBUG/",
                              "/Users/apple/Projects/openMVS/build/bin/");
            }
            ImGuiFileDialog::Instance()->Close();
        }
    }
}

auto PipelineModule::update(float delta_time) -> bool {
    if (!opengl_ready && pipeline.state == PipelineState::GLOBAL_SFM) {
        program = link(compile(GL_VERTEX_SHADER, "shaders/vertex.glsl"),
                       compile(GL_FRAGMENT_SHADER, "shaders/fragment.glsl"));

        load_ply_as_pointcloud("products/sfm/cloud_and_poses.ply");
        
        eye = glm::vec3(0.0f, 0.0f, 5.0f);
        center = glm::vec3(0.0f);
        perspective_mat = glm::perspective(glm::radians(45.0f), (float) window_size.x / window_size.y, 0.01f, 200.0f);
        view_mat = glm::lookAt(glm::vec3(0.0f, 0.0f, 5.0f), glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        render_state = pipeline.state;

        opengl_ready = true;
        time = 0.0f;
    }
    if (opengl_ready && render_state != pipeline.state && pipeline.state == PipelineState::STRUCTURE_FROM_KNOWN_POSES) {
        render_state = pipeline.state;
        load_colorized_ply_as_pointcloud("products/sfm/colorized.ply");
    }
    if (opengl_ready && render_state != pipeline.state && pipeline.state == PipelineState::MVG2MVS) {
        render_state = pipeline.state;
        load_colorized_ply_as_pointcloud("products/sfm/robust_colorized.ply");
    }
    if (opengl_ready && render_state != pipeline.state && pipeline.state == PipelineState::RECONSTRUCT_MESH) {
        render_state = pipeline.state;
        load_colorized_ply_as_pointcloud("products/mvs/scene_dense.ply");
    }
    if (opengl_ready && render_state != pipeline.state && pipeline.state == PipelineState::REFINE_MESH) {
        render_state = pipeline.state;
        load_ply_as_mesh("products/mvs/scene_dense_mesh.ply");
    }
    if (opengl_ready && render_state != pipeline.state && pipeline.state == PipelineState::FINISHED_SUCCESS) {
        render_state = pipeline.state;
        load_ply_and_texture_map("products/mvs/scene_dense_mesh_refine_texture.ply",
                                 "products/mvs/scene_dense_mesh_refine_texture.png");
    }
    time += delta_time;

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
    return opengl_ready;
}

auto PipelineModule::list_images(std::filesystem::path path) -> int {
    auto legit_image = 0;
    image_listing.clear();
    for (const auto &entry : std::filesystem::directory_iterator(path)) {
        const auto fext = entry.path().extension();
        if (fext != ".jpg" && fext != ".bmp" && fext != ".png" && fext != ".jpeg") {
            continue;
        }
        auto extension_fix = entry.path();
        if (fext == ".jpeg") {
            extension_fix.replace_extension(".jpg");
            std::rename(entry.path().c_str(), extension_fix.c_str());
        }
        image_listing.push_back(extension_fix.string());
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
    glPointSize(5.0f);
    glDrawArrays(render_mode, 0, num_vertices);
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
    std::vector<Vertex> verts(vertices->count);
    
    for (auto i = 0; i < vertices->count; i++) {
        Vertex vertex;
        vertex = {
            point_fetch(vertices, i),
            { 0.0f, 0.0f, 0.0f },
            { 0.0f, 0.0f },
            { 1.0f, 0.5f, 0.0f }
        };
        verts[i] = vertex;
    }
    glm::vec3 center_of_gravity = glm::vec3(0.0f);
    for (auto i = 0; i < verts.size(); i++) {
        center_of_gravity += verts[i].position;
    }
    center_of_gravity /= verts.size();
    for (auto i = 0; i < verts.size(); i++) {
        verts[i].position -= center_of_gravity;
    }

    setup_render(verts, GL_POINTS);
}

auto PipelineModule::load_colorized_ply_as_pointcloud(std::string path) -> void { 
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
    catch (const std::exception & e) { RECON_LOG(PIPELINE) << "tinyply 错误: " << e.what(); }
    
    try { colors = file.request_properties_from_element("vertex", { "red", "green", "blue" }); }
    catch (const std::exception & e) { RECON_LOG(PIPELINE) << "tinyply 无法读取颜色通道 (red, green, blue): " << e.what(); }

    try { colors = file.request_properties_from_element("vertex", { "r", "g", "b" }); }
    catch (const std::exception & e) { RECON_LOG(PIPELINE) << "tinyply 无法读取颜色通道 (r, g, b): " << e.what(); }
    
    file.read(reader);
    reader.close();

    RECON_LOG(PIPELINE) << "读取完毕。节点数量：" << vertices->count;
    std::vector<Vertex> verts(vertices->count);

    for (auto i = 0; i < vertices->count; i++) {
        Vertex vertex;
        vertex = {
            point_fetch(vertices, i),
            { 0.0f, 0.0f, 0.0f },
            { 0.0f, 0.0f },
            point_fetch(colors, i)
        };
        verts[i] = vertex;
    }

    glm::vec3 center_of_gravity = glm::vec3(0.0f);
    for (auto i = 0; i < verts.size(); i++) {
        center_of_gravity += verts[i].position;
    }
    center_of_gravity /= verts.size();
    for (auto i = 0; i < verts.size(); i++) {
        verts[i].position -= center_of_gravity;
    }

    setup_render(verts, GL_POINTS);
}

auto PipelineModule::load_ply_as_mesh(std::string path) -> void {
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
    catch (const std::exception & e) { RECON_LOG(PIPELINE) << "tinyply 错误: " << e.what(); }

    try { faces = file.request_properties_from_element("face", { "vertex_indices" }, 3); }
    catch (const std::exception & e) { RECON_LOG(PIPELINE) << "tinyply 无法读取面: " << e.what(); }
    
    file.read(reader);
    reader.close();

    RECON_LOG(PIPELINE) << "读取完毕。节点数量：" << vertices->count << "，面数量：" << faces->count;
    std::vector<Vertex> verts;
    glm::u32vec3 *indices = (glm::u32vec3 *) faces->buffer.get();
    for (auto i = 0; i < faces->count; i++) {
        glm::u32vec3 &triangle = indices[i];
        Vertex v1, v2, v3;
        v1 = {
            point_fetch(vertices, triangle.x),
            { 0.0f, 0.0f, 0.0f },
            { 0.0f, 0.0f },
            { 1.0f, 0.5f, 0.0f }
        };
        v2 = {
            point_fetch(vertices, triangle.y),
            { 0.0f, 0.0f, 0.0f },
            { 0.0f, 0.0f },
            { 1.0f, 0.5f, 0.0f }
        };
        v3 = {
            point_fetch(vertices, triangle.z),
            { 0.0f, 0.0f, 0.0f },
            { 0.0f, 0.0f },
            { 1.0f, 0.5f, 0.0f }
        };
        verts.push_back(v1);
        verts.push_back(v2);
        verts.push_back(v3);
    }

    glm::vec3 center_of_gravity = glm::vec3(0.0f);
    for (auto i = 0; i < verts.size(); i++) {
        center_of_gravity += verts[i].position;
    }
    center_of_gravity /= verts.size();
    for (auto i = 0; i < verts.size(); i++) {
        verts[i].position -= center_of_gravity;
    }
    RECON_LOG(PIPELINE) << "重心：" << center_of_gravity.x << ", " << center_of_gravity.y << ", " << center_of_gravity.z;

    setup_render(verts, GL_TRIANGLES);
}

auto PipelineModule::load_ply_and_texture_map(std::string path, std::string texture_path) -> void {
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
    catch (const std::exception & e) { RECON_LOG(PIPELINE) << "tinyply 错误: " << e.what(); }

    try { faces = file.request_properties_from_element("face", { "vertex_indices" }, 3); }
    catch (const std::exception & e) { RECON_LOG(PIPELINE) << "tinyply 无法读取面: " << e.what(); }
    
    try { tex_coords = file.request_properties_from_element("face", { "texcoord" }, 6); }
    catch (const std::exception & e) { RECON_LOG(PIPELINE) << "tinyply 错误: " << e.what(); }
    
    file.read(reader);
    reader.close();

    RECON_LOG(PIPELINE) << "读取完毕。节点数量：" << vertices->count << "，面数量：" << faces->count;
    std::vector<Vertex> verts;
    glm::u32vec3 *indices = (glm::u32vec3 *) faces->buffer.get();
    float *uv_arr = (float *) tex_coords->buffer.get();
    for (auto i = 0; i < faces->count; i++) {
        glm::u32vec3 &triangle = indices[i];
        Vertex v1, v2, v3;
        v1 = {
            point_fetch(vertices, triangle.x),
            { 0.0f, 0.0f, 0.0f },
            { uv_arr[i * 6 + 0], uv_arr[i * 6 + 1] },
            { 1.0f, 0.5f, 0.0f }
        };
        v2 = {
            point_fetch(vertices, triangle.y),
            { 0.0f, 0.0f, 0.0f },
            { uv_arr[i * 6 + 2], uv_arr[i * 6 + 3] },
            { 1.0f, 0.5f, 0.0f }
        };
        v3 = {
            point_fetch(vertices, triangle.z),
            { 0.0f, 0.0f, 0.0f },
            { uv_arr[i * 6 + 4], uv_arr[i * 6 + 5] },
            { 1.0f, 0.5f, 0.0f }
        };
        verts.push_back(v1);
        verts.push_back(v2);
        verts.push_back(v3);
    }

    glm::vec3 center_of_gravity = glm::vec3(0.0f);
    for (auto i = 0; i < verts.size(); i++) {
        center_of_gravity += verts[i].position;
    }
    center_of_gravity /= verts.size();
    for (auto i = 0; i < verts.size(); i++) {
        verts[i].position -= center_of_gravity;
    }
    RECON_LOG(PIPELINE) << "重心：" << center_of_gravity.x << ", " << center_of_gravity.y << ", " << center_of_gravity.z;
    
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
    auto *data = stbi_load(texture_path.c_str(), &width, &height, &channels, 0);
    if (!data) {
        RECON_LOG(PIPELINE) << "加载材质失败：" << texture_path << " 未找到或无权限";
        return;
    }
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
    stbi_image_free(data);
    glBindTexture(GL_TEXTURE_2D, GL_NONE);

    setup_render(verts, GL_TRIANGLES);
}

auto PipelineModule::setup_render(std::vector<Vertex> vertices, GLuint render_mode) -> void { 
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
    
    this->render_mode = render_mode;
    num_vertices = (int) vertices.size();
}

