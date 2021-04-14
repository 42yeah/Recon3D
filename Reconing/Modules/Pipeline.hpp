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
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <filesystem>
#include <mutex>
#include <map>

// O P E N M V G (S) ///////////////////////////
#define OPENMVG_STD_UNORDERED_MAP
#define OPENMVG_USE_OPENMP
#include <openMVG/sfm/sfm_data.hpp>
#include <openMVG/image/image_io.hpp>
#include <Eigen/Geometry>
#include <openMVG/matching/indMatch.hpp>
#include <openMVG/sfm/pipelines/sfm_features_provider.hpp>
#include <openMVG/sfm/pipelines/sfm_matches_provider.hpp>

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::sfm;
using namespace openMVG::image;
using namespace openMVG::matching;
using namespace openMVG::features;


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
    COLORIZING = 6
};

enum class PairMode {
    EXHAUSITIVE = 0
};

enum class GeometricModel {
    FUNDAMENTAL_MATRIX = 0,
    ESSENTIAL_MATRIX = 1
};

class Pipeline {
public:
    Pipeline() : state(PipelineState::INTRINSICS_ANALYSIS) {}

    Pipeline(std::vector<std::string> image_listing, std::filesystem::path base_path) : image_listing(image_listing),
        base_path(base_path),
        state(PipelineState::INTRINSICS_ANALYSIS),
        progress(0.0f) {};

    auto run() -> bool;
    
    auto path_of_view(const View &view) -> std::filesystem::path;
    
    auto save_sfm(const std::string path) -> bool;
    
    auto export_to_ply(const std::string path,
                       std::vector<Vec3> vertices,
                       std::vector<Vec3> camera_poses,
                       std::vector<Vec3> colored_points = std::vector<Vec3>()) -> bool;

    // P I P E L I N E ///////////////////////////////
    auto intrinsics_analysis() -> bool;
    
    auto feature_detection() -> bool;
    
    auto match_features() -> bool;
    
    auto incremental_sfm() -> bool;
    
    auto global_sfm() -> bool;
    
    auto colorize() -> bool;

    PipelineState state;
    float progress;

private:
    auto mkdir_if_not_exists(std::filesystem::path path) -> void;
    
    // P R O V I D E R S //////////////////////////////
    std::shared_ptr<Features_Provider> features_provider;
    std::shared_ptr<Matches_Provider> matches_provider;
    
    // D A T A ////////////////////////////////////////
    std::vector<std::string> image_listing;
    std::filesystem::path base_path;
    std::map<int, Image<unsigned char> > images;
    PairWiseMatches matches;
    
    SfM_Data sfm_data;
};

};


/// Files to load when states change:
/// 1. AFTER incremental SfM, we can load the initial PLY;
/// 2. AFTER global SfM, we can simply load it again.
/// 3. AFTER data color calculation, we can load it, this time with color!
class PipelineModule : public Module {
public:
    PipelineModule() : Module(PIPELINE),
        state(PipelineNS::State::ASKING_FOR_INPUT),
        image_listing(std::vector<std::string>()),
        VAO(0), VBO(0), program(0), opengl_ready(false), time(0.0f), radius(5.0f) {}
    
    virtual auto update(float delta_time) -> void override;
    
    virtual auto update_ui() -> void override;
    
    virtual auto render() -> void override;

    virtual auto list_images(std::filesystem::path path) -> int;
    
private:
    auto load_ply_as_pointcloud(std::string path) -> void;

    PipelineNS::State state;
    PipelineNS::Pipeline pipeline;
    
    // I N P U T S //////////////////////////////////
    std::vector<std::string> image_listing;
    
    // O P E N G L //////////////////////////////////
    bool opengl_ready;
    int num_vertices;
    GLuint VAO, VBO, program;
    glm::vec3 eye, center;
    glm::mat4 view_mat, perspective_mat;
    float time, radius;
};

#endif /* Pipeline_hpp */
