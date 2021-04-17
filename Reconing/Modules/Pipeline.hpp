//
//  Pipeline.hpp
//  Reconing
//
//  Created by apple on 12/04/2021.
//

#ifndef Pipeline_hpp
#define Pipeline_hpp

#define PIPELINE "管线"

#include "common.hpp"
#include "Module.hpp"
#include <vector>
#include <chrono>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <filesystem>
#include <iostream>
#include <functional>
#include <mutex>
#include <map>
#include <tinyply.h>


namespace PipelineNS {

enum class State {
    ASKING_FOR_INPUT = 0,
    CHOOSING_FILE = 1,
    FOLDER_CHOSEN = 2,
    RUNNING = 3
};

enum class PipelineState {
    FINISHED_ERR = -1,
    FINISHED_SUCCESS = 0,
    INTRINSICS_ANALYSIS = 1,
    FEATURE_DETECTION = 2,
    MATCHING_FEATURES = 3,
    INCREMENTAL_SFM = 4,
    GLOBAL_SFM = 5,
    COLORIZING = 6,
    STRUCTURE_FROM_KNOWN_POSES = 7,
    COLORIZED_ROBUST_TRIANGULATION = 8,
    MVG2MVS = 9,
    DENSIFY_PC = 10,
    RECONSTRUCT_MESH = 11,
    REFINE_MESH = 12,
    TEXTURE_MESH = 13,
    NUM_PROCEDURES
};

template<typename ...Ar>
auto invoke(std::function<void(std::string)> callback,
            Ar... cmd) -> bool;

auto point_fetch(std::shared_ptr<tinyply::PlyData> ply_data, int offset) -> glm::vec3;

class Pipeline {
public:
    Pipeline() : state(PipelineState::FINISHED_SUCCESS) {}

    Pipeline(std::vector<std::string> image_listing, std::filesystem::path base_path,
             std::string mvg_executable_path,
             std::string mvs_executable_path) {
        init(image_listing, base_path, mvg_executable_path, mvs_executable_path);
    }
    
    auto init(std::vector<std::string> image_listing, std::filesystem::path base_path,
              std::string mvg_executable_path,
              std::string mvs_executable_path) -> void;

    auto run() -> bool;
    
    auto export_to_ply(const std::string path,
                       std::vector<glm::vec3> vertices,
                       std::vector<glm::vec3> camera_poses,
                       std::vector<glm::vec3> colored_points = std::vector<glm::vec3>()) -> bool;
    
    auto save_session(std::string name) -> bool;

    // P I P E L I N E ///////////////////////////////
    auto intrinsics_analysis() -> bool;
    
    auto feature_detection() -> bool;
    
    auto match_features() -> bool;
    
    auto incremental_sfm() -> bool;
    
    auto global_sfm() -> bool;
    
    auto colorize(PipelineState state) -> bool;
    
    auto structure_from_known_poses() -> bool;
    
    auto export_openmvg_to_openmvs() -> bool;
    
    auto density_pointcloud() -> bool;
    
    auto reconstruct_mesh() -> bool;
    
    auto refine_mesh() -> bool;
    
    auto texture_mesh() -> bool;
    

    PipelineState state;
    float progress;

private:
    auto mkdir_if_not_exists(std::filesystem::path path) -> void;
    
    auto mvg() -> std::filesystem::path;
    
    auto mvs() -> std::filesystem::path;
    
    // D A T A ////////////////////////////////////////
    std::vector<std::string> image_listing;
    std::filesystem::path base_path;
    std::string mvg_executable_path;
    std::string mvs_executable_path;
};

struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec2 tex_coord;
    glm::vec3 color;
};

};


/// Files to load when states change:
/// 1. AFTER incremental SfM, we can load the initial PLY;
/// 2. AFTER global SfM, we can simply load it again.
/// 3. AFTER data color calculation, we can load it, this time with color!
class PipelineModule : public Module {
public:
    PipelineModule() : Module(PIPELINE),
        state(PipelineNS::State::RUNNING),
        image_listing(std::vector<std::string>()),
        VAO(0), VBO(0), program(0), opengl_ready(false), time(0.0f), radius(5.0f),
        horizontal_rotation_target(0.0f), horizontal_rotation(0.0f),
        center(0.0f, 0.0f, 0.0f),
        render_mode(GL_POINTS),
        mesh_texture(GL_NONE) {
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            std::memset(session_name, 0, sizeof(session_name));
            auto t_struct = *localtime(&time_t);
            strftime(session_name, sizeof(session_name), "recon-at-%Y-%m-%d-%H-%M-%S", &t_struct);
        }
    
    virtual auto update(float delta_time) -> void override;
    
    virtual auto update_ui() -> void override;
    
    virtual auto render() -> void override;

    virtual auto list_images(std::filesystem::path path) -> int;
    
private:
    auto load_ply_as_pointcloud(std::string path) -> void;
    
    auto load_colorized_ply_as_pointcloud(std::string path) -> void;
    
    auto load_ply_as_mesh(std::string path) -> void;
    
    auto load_ply_and_texture_map(std::string path, std::string texture_path) -> void;
    
    auto setup_render(std::vector<PipelineNS::Vertex> vertices, GLuint render_mode) -> void;

    PipelineNS::PipelineState render_state;
    PipelineNS::State state;
    PipelineNS::Pipeline pipeline;
    
    // I N P U T S //////////////////////////////////
    std::vector<std::string> image_listing;
    
    // O P E N G L //////////////////////////////////
    bool opengl_ready;
    int num_vertices;
    GLuint VAO, VBO, program;
    glm::vec3 eye, center;
    glm::mat4 model_mat, view_mat, perspective_mat;
    float time, radius, horizontal_rotation, horizontal_rotation_target;
    GLuint render_mode;
    
    GLuint mesh_texture;
    char session_name[512];
};

#endif /* Pipeline_hpp */
