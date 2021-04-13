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

// O P E N M V G (S) ///////////////////////////
#include <openMVG/cameras/cameras.hpp>
#include <openMVG/exif/exif_IO_EasyExif.hpp>
#include <openMVG/geodesy/geodesy.hpp>
#include <openMVG/numeric/eigen_alias_definition.hpp>
#include <openMVG/sfm/sfm_data_io.hpp>
#include <openMVG/sfm/sfm_data_utils.hpp>
#include <openMVG/sfm/sfm_view.hpp>
#include <openMVG/sfm/sfm_view_priors.hpp>
#include <openMVG/types.hpp>
#include <nonFree/sift/SIFT_describer_io.hpp>
#include <openMVG/graph/graph.hpp>
#include <openMVG/graph/graph_stats.hpp>
#include <openMVG/features/descriptor.hpp>
#include <openMVG/features/feature.hpp>
#include <openMVG/matching/indMatch.hpp>
#include <openMVG/matching/indMatch_utils.hpp>
#include <openMVG/matching_image_collection/Matcher_Regions.hpp>
#include <openMVG/matching_image_collection/Cascade_Hashing_Matcher_Regions.hpp>
#include <openMVG/matching_image_collection/GeometricFilter.hpp>
#include <openMVG/sfm/pipelines/sfm_features_provider.hpp>
#include <openMVG/sfm/pipelines/sfm_regions_provider.hpp>
#include <openMVG/sfm/pipelines/sfm_regions_provider_cache.hpp>
#include <openMVG/matching_image_collection/F_ACRobust.hpp>
#include <openMVG/matching_image_collection/E_ACRobust.hpp>
#include <openMVG/matching_image_collection/E_ACRobust_Angular.hpp>
#include <openMVG/matching_image_collection/Eo_Robust.hpp>
#include <openMVG/matching_image_collection/H_ACRobust.hpp>
#include <openMVG/matching_image_collection/Pair_Builder.hpp>
#include <openMVG/matching/pairwiseAdjacencyDisplay.hpp>
#include <openMVG/stl/stl.hpp>

using namespace openMVG::exif;
using namespace openMVG::geodesy;
using namespace openMVG::features;
using namespace openMVG::matching;
using namespace openMVG::robust;
using namespace openMVG::matching_image_collection;


// P I P E L I N E ///////////////////////////
#include <glm/glm.hpp>

using namespace PipelineNS;

std::mutex mutex;

auto run_pipeline(Pipeline *pipeline_ptr) -> void {
    auto &pipeline = *pipeline_ptr;
    
    pipeline.run();
}

auto Pipeline::run() -> bool {
    mkdir_if_not_exists("products");
    mkdir_if_not_exists("products/features");
    mkdir_if_not_exists("products/matches");
    if (intrinsics_analysis() &&
        feature_detection() &&
        match_features()) {
        return true;
    }
    mutex.lock();
    LOG(PIPELINE) << "管线运行错误。";
    mutex.unlock();
    state = PipelineState::FINISHED_ERR;
    return false;
}

auto Pipeline::intrinsics_analysis() -> bool {
    progress = 0.0f;
    state = PipelineState::INTRINSICS_ANALYSIS;
    mutex.lock();
    LOG(PIPELINE) << "相机内部参数提取开始。";
    mutex.unlock();
    sfm_data.s_root_path = base_path.string();
    Views &views = sfm_data.views;
    Intrinsics &intrinsics = sfm_data.intrinsics;
    for (auto i = 0; i < image_listing.size(); i++) {
        const auto &entry = image_listing[i];
        progress = (float) i / image_listing.size();
        auto width = -1.0, height = -1.0, ppx = -1.0, ppy = -1.0, focal = 5.0;
        ImageHeader header;
        if (!ReadImageHeader(entry.c_str(), &header)) {
            mutex.lock();
            LOG(PIPELINE) << "无法读取图片头：" << entry << ", 跳过。";
            mutex.unlock();
            continue;
        }
        width = header.width;
        height = header.height;
        ppx = width / 2.0;
        ppy = height / 2.0;
        const auto exif_reader = new Exif_IO_EasyExif();
        exif_reader->open(entry);
        if (exif_reader->doesHaveExifInfo()) {
            auto exif_focal = exif_reader->getFocal();
            if (exif_focal != 0.0f) {
                focal = exif_focal;
            }
        }
        std::shared_ptr<IntrinsicBase> intrinsic = std::make_shared<Pinhole_Intrinsic_Radial_K3>(width, height, focal, ppx, ppy, 0.0, 0.0, 0.0);
        auto iterator = image_listing.begin() + i;
        View view(*iterator, (int) views.size(), (int) views.size(), (int) views.size(), width, height);
        intrinsics[view.id_view] = intrinsic;
        views[view.id_view] = std::make_shared<View>(view);
    }
    mutex.lock();
    LOG(PIPELINE) << "相机内部参数提取完成。";
    mutex.unlock();
    return true;
}

auto Pipeline::feature_detection() -> bool {
    progress = 0.0f;
    state = PipelineState::FEATURE_DETECTION;
    mutex.lock();
    LOG(PIPELINE) << "开始特征提取...";
    mutex.unlock();
    auto image_describer = new SIFT_Image_describer(SIFT_Image_describer::Params());
    
    for (auto i = 0; i < sfm_data.views.size(); i++) {
        progress = (float) i / sfm_data.views.size();
        auto iter_views = sfm_data.views.begin();
        std::advance(iter_views, i);
        const auto *view = iter_views->second.get();
        
        Image<unsigned char> image;
        if (!ReadImage(image_listing[i].c_str(), &image)) {
            mutex.lock();
            LOG(PIPELINE) << "图片读取失败：" << image_listing[i] << "，跳过。";
            mutex.unlock();
        }
        Image<unsigned char> *mask = nullptr;
        images[view->id_view] = image;
        auto regions = image_describer->Describe(image, mask);

        auto pov = path_of_view(*view).replace_extension("");
        if (regions && !image_describer->Save(regions.get(),
                                                     pov.string() + ".feat",
                                                     pov.string() + ".desc")) {
            mutex.lock();
            LOG(PIPELINE) << "无法为图片提取特征点：" << image_listing[i] << "。管线任务失败。";
            progress = 0.0f;
            mutex.unlock();
            return false;
        }
    }
    mutex.lock();
    LOG(PIPELINE) << "图片特征点提取完成。";
    mutex.unlock();
    return true;
}

auto Pipeline::match_features() -> bool {
    progress = 0.0f;
    state = PipelineState::MATCHING_FEATURES;
    mutex.lock();
    LOG(PIPELINE) << "开始特征匹配，使用方法：HNSWL2，距离比：0.8，几何模型：基础矩阵。";
    mutex.unlock();

    const auto dist_ratio = 0.8f;
    const auto method = HNSW_L2;
    const auto pair_mode = PairMode::EXHAUSITIVE;
    const auto geometric_model = GeometricModel::FUNDAMENTAL_MATRIX;
    std::string file_name = "";
    switch (geometric_model) {
        case GeometricModel::FUNDAMENTAL_MATRIX:
            file_name = "matches.f.bin";
            break;
            
        case GeometricModel::ESSENTIAL_MATRIX:
            file_name = "matches.e.bin";
            break;
    }
    
    std::unique_ptr<Regions> regions;
    regions.reset(new SIFT_Regions());
    auto regions_provider = std::make_shared<Regions_Provider>();
    if (!regions_provider->load(sfm_data, "products/features", regions, nullptr)) {
        LOG(PIPELINE) << "区间错误。";
        return false;
    }
    
    PairWiseMatches putative_matches;
    std::vector<glm::ivec2> image_sizes;
    image_sizes.resize(images.size());
    
    const auto views = sfm_data.GetViews();
    for (auto it = views.begin(); it != views.end(); it++) {
        const auto *view = it->second.get();
        image_sizes.emplace_back(view->ui_width, view->ui_height);
    }

    mutex.lock();
    LOG(PIPELINE) << "正在开始进行推断匹配...";
    mutex.unlock();
    
    std::unique_ptr<Matcher> collection_matcher;
    collection_matcher.reset(new Matcher_Regions(dist_ratio, method));
    
    Pair_Set pairs;
    switch (pair_mode) {
        case PairMode::EXHAUSITIVE:
            pairs = exhaustivePairs(images.size());
            break;
    }
    collection_matcher->Match(regions_provider, pairs, putative_matches);
    
//    PairWiseMatchingToAdjacencyMatrixSVG(images.size(), putative_matches, "products/matches/putative_adjacency_mat.svg");
    
    // F I L T E R ///////////////////////////////////////////////////
    std::unique_ptr<ImageCollectionGeometricFilter> filterer(new ImageCollectionGeometricFilter(&sfm_data, regions_provider));
    const double d_distance_ratio = 0.6;
    PairWiseMatches geometric_matches;
    switch (geometric_model) {
        case GeometricModel::FUNDAMENTAL_MATRIX:
            filterer->Robust_model_estimation(GeometricFilter_FMatrix_AC(4.0, 2048),
                                              putative_matches,
                                              false, // Guided matching
                                              d_distance_ratio);
            geometric_matches = filterer->Get_geometric_matches();
            break;
            
        case GeometricModel::ESSENTIAL_MATRIX:
            filterer->Robust_model_estimation(GeometricFilter_EMatrix_AC(4.0, 2048),
                                              putative_matches,
                                              false,
                                              d_distance_ratio);
            geometric_matches = filterer->Get_geometric_matches();
            std::vector<PairWiseMatches::key_type> to_remove;
            for (const auto &match : geometric_matches) {
                const auto putative_photometric_count = putative_matches.find(match.first)->second.size();
                const auto putative_geometric_count = match.second.size();
                const auto ratio = (float) putative_geometric_count / putative_photometric_count;
                if (putative_geometric_count < 50 || ratio < 0.3f) {
                    to_remove.push_back(match.first);
                }
            }
            for (const auto &k : to_remove) {
                geometric_matches.erase(k);
            }
            break;
    }
    mutex.lock();
    LOG(PIPELINE) << "匹配结束，正在保存...";
    if (!Save(geometric_matches, std::string("products/matches/") + file_name)) {
        LOG(PIPELINE) << "保存失败：" << (std::string("products/matches/") + file_name);
        mutex.unlock();
        return false;
    }
    mutex.unlock();
    return true;
}

auto Pipeline::path_of_view(const openMVG::sfm::View &view) -> std::filesystem::path {
    std::filesystem::path img_path = view.s_Img_path;
    return std::filesystem::path("products/features") / img_path.filename();
}

auto Pipeline::mkdir_if_not_exists(std::filesystem::path path) -> void { 
    if (!std::filesystem::exists(path)) {
        std::filesystem::create_directory(path);
    }
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
                }
                mutex.unlock();
                ImGui::ProgressBar(pipeline.progress);
                break;
        }
        
        ImGui::End();
    }
    if (state == State::CHOOSING_FILE) {
        ImGuiFileDialog::Instance()->OpenDialog("Folder", "选择输入目录...", nullptr, ".");
        ImGui::SetNextWindowSize({ 300, 300 }, ImGuiCond_FirstUseEver);
        if (ImGuiFileDialog::Instance()->Display("Folder")) {
            if (ImGuiFileDialog::Instance()->IsOk()) {
                std::string path = ImGuiFileDialog::Instance()->GetCurrentPath();
                LOG(PIPELINE) << "目录已选定：" << path;
                LOG(PIPELINE) << "有效数据：" <<
                    list_images(path);
                state = State::FOLDER_CHOSEN;
                pipeline = Pipeline(image_listing, path);
            }
            ImGuiFileDialog::Instance()->Close();
        }
    }
}

auto PipelineModule::update() -> void { 
    
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

