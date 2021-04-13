//
//  Pipeline.hpp
//  Reconing
//
//  Created by apple on 12/04/2021.
//

#ifndef Pipeline_hpp
#define Pipeline_hpp

#define PIPELINE "管线"

#include "Module.hpp"
#include "common.hpp"
#include <vector>
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

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::sfm;
using namespace openMVG::image;
using namespace openMVG::matching;


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
    INCREMENTAL_SFM = 4
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

    // P I P E L I N E ///////////////////////////////
    auto intrinsics_analysis() -> bool;
    
    auto feature_detection() -> bool;
    
    auto match_features() -> bool;
    
    auto incremental_sfm() -> bool;
    

    PipelineState state;
    float progress;

private:
    std::vector<std::string> image_listing;
    std::filesystem::path base_path;
    std::map<int, Image<unsigned char> > images;
    PairWiseMatches matches;
    
    
    auto mkdir_if_not_exists(std::filesystem::path path) -> void;
    
    SfM_Data sfm_data;
};

};


/// Pipeline procedure:
/// 1. Intrinsics analysis
class PipelineModule : public Module {
public:
    PipelineModule() : Module(PIPELINE),
        state(PipelineNS::State::ASKING_FOR_INPUT),
        image_listing(std::vector<std::string>()) {}
    
    virtual auto update() -> void override;
    
    virtual auto update_ui() -> void override;

    virtual auto list_images(std::filesystem::path path) -> int;
    
private:
    PipelineNS::State state;
    PipelineNS::Pipeline pipeline;
    
    // I N P U T S //////////////////////////////////
    std::vector<std::string> image_listing;
};

#endif /* Pipeline_hpp */
