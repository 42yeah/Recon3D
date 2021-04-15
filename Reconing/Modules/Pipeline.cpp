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

// O P E N M V G ///////////////////////////////
#include <openMVG/cameras/cameras.hpp>
#include <openMVG/exif/exif_IO_EasyExif.hpp>
#include <openMVG/geodesy/geodesy.hpp>
#include <openMVG/numeric/eigen_alias_definition.hpp>
#include <openMVG/sfm/sfm_data.hpp>
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
#include <openMVG/matching/indMatch_utils.hpp>
#include <openMVG/matching_image_collection/Matcher_Regions.hpp>
#include <openMVG/matching_image_collection/Cascade_Hashing_Matcher_Regions.hpp>
#include <openMVG/matching_image_collection/GeometricFilter.hpp>
#include <openMVG/sfm/pipelines/global/GlobalSfM_rotation_averaging.hpp>
#include <openMVG/sfm/pipelines/global/GlobalSfM_translation_averaging.hpp>
#include <openMVG/sfm/pipelines/global/sfm_global_engine_relative_motions.hpp>
#include <openMVG/sfm/pipelines/sequential/sequential_SfM.hpp>
#include <openMVG/sfm/sfm_data_colorization.hpp>
#include <openMVG/sfm/sfm_data_BA.hpp>
#include <openMVG/sfm/sfm_data_BA_ceres.hpp>
#include <openMVG/sfm/sfm_data_filters.hpp>
#include <openMVG/sfm/sfm_data_filters_frustum.hpp>
#include <openMVG/sfm/pipelines/structure_from_known_poses/structure_estimator.hpp>
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
using namespace openMVG::robust;
using namespace openMVG::matching_image_collection;

// O P E N M V S ////////////////////////////
#define _USE_EIGEN
#include <MVS/Interface.h>


// P I P E L I N E ///////////////////////////
#define TINYPLY_IMPLEMENTATION
#include <tinyply.h>
#include <GLFW/glfw3.h>

using namespace PipelineNS;

std::mutex mutex;

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

auto Pipeline::path_of_view(const openMVG::sfm::View &view) -> std::filesystem::path {
    std::filesystem::path img_path = view.s_Img_path;
    return std::filesystem::path("products/features") / img_path.filename();
}

auto Pipeline::mkdir_if_not_exists(std::filesystem::path path) -> void {
    if (!std::filesystem::exists(path)) {
        std::filesystem::create_directory(path);
    }
}

auto Pipeline::save_sfm(const std::string path) -> bool {
    mutex.lock();
    RECON_LOG(PIPELINE) << "正在保存 SfM 数据到 " << path << "...";
    auto success = Save(sfm_data, path, ESfM_Data(VIEWS | INTRINSICS));
    mutex.unlock();
    return success;
}

auto Pipeline::export_to_ply(const std::string path, std::vector<Vec3> vertices, std::vector<Vec3> camera_poses, std::vector<Vec3> colored_points) -> bool {
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
            writer << vertices[i](0) << ' '
                << vertices[i](1) << ' '
                << vertices[i](2) << ' '
                << (int) color(0) << ' '
                << (int) color(1) << ' '
                << (int) color(2) << std::endl;
        } else {
            writer << vertices[i](0) << ' '
                << vertices[i](1) << ' '
                << vertices[i](2) << ' '
                << 255 << ' '
                << 255 << ' '
                << 255 << std::endl;
        }
    }
    
    for (auto i = 0; i < camera_poses.size(); i++) {
        writer << camera_poses[i](0) << ' '
            << camera_poses[i](1) << ' '
            << camera_poses[i](2) << ' '
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
    sfm_data.s_root_path = base_path.string();
    Views &views = sfm_data.views;
    Intrinsics &intrinsics = sfm_data.intrinsics;
    for (auto i = 0; i < image_listing.size(); i++) {
        const auto &entry = image_listing[i];
        progress = (float) i / image_listing.size();
        auto width = -1.0, height = -1.0, ppx = -1.0, ppy = -1.0, focal = 2884.0;
        ImageHeader header;
        if (!ReadImageHeader(entry.c_str(), &header)) {
            mutex.lock();
            RECON_LOG(PIPELINE) << "无法读取图片头：" << entry << ", 跳过。";
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
    sfm_data.s_root_path = "";
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
    auto image_describer = new SIFT_Image_describer(SIFT_Image_describer::Params());
    
    for (auto i = 0; i < sfm_data.views.size(); i++) {
        progress = (float) i / sfm_data.views.size();
        auto iter_views = sfm_data.views.begin();
        std::advance(iter_views, i);
        const auto *view = iter_views->second.get();
        
        Image<unsigned char> image;
        if (!ReadImage(image_listing[i].c_str(), &image)) {
            mutex.lock();
            RECON_LOG(PIPELINE) << "图片读取失败：" << image_listing[i] << "，跳过。";
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
            RECON_LOG(PIPELINE) << "无法为图片提取特征点：" << image_listing[i] << "。管线任务失败。";
            progress = 0.0f;
            mutex.unlock();
            return false;
        }
    }
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
    
    std::unique_ptr<Regions> regions(new SIFT_Regions());
    auto regions_provider = std::make_shared<Regions_Provider>();
    if (!regions_provider->load(sfm_data, "products/features", regions, nullptr)) {
        RECON_LOG(PIPELINE) << "区间错误。";
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
    RECON_LOG(PIPELINE) << "正在开始进行推断匹配...";
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

    // F I L T E R ///////////////////////////////////////////////////
    std::unique_ptr<ImageCollectionGeometricFilter> filterer(new ImageCollectionGeometricFilter(&sfm_data, regions_provider));
    const double d_distance_ratio = 0.6;
    PairWiseMatches geometric_matches;
    progress = 0.5f;
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
    progress = 0.97f;
    RECON_LOG(PIPELINE) << "匹配结束，正在保存...";
    if (!Save(geometric_matches, std::string("products/matches/") + file_name)) {
        RECON_LOG(PIPELINE) << "保存失败：" << (std::string("products/matches/") + file_name);
        mutex.unlock();
        return false;
    }
    mutex.unlock();
    return true;
}

auto Pipeline::incremental_sfm() -> bool {
    progress = 0.0f;
    state = PipelineState::INCREMENTAL_SFM;
    mutex.lock();
    RECON_LOG(PIPELINE) << "开始进行初步 SfM (Structure from Motion)。";
    mutex.unlock();
    
    const auto triangulation_method = ETriangulationMethod::DEFAULT;
    const auto camera_model = PINHOLE_CAMERA_RADIAL3;
    const auto resection_method = resection::SolverType::DEFAULT;
    const auto intrinsic_refinement_options = cameras::Intrinsic_Parameter_Type::ADJUST_FOCAL_LENGTH |
        cameras::Intrinsic_Parameter_Type::ADJUST_PRINCIPAL_POINT |
        cameras::Intrinsic_Parameter_Type::ADJUST_DISTORTION;
    
    std::unique_ptr<Regions> regions(new SIFT_Regions());
    features_provider = std::make_shared<Features_Provider>();
    
    progress = 0.1f;
    if (!features_provider->load(sfm_data, "products/features", regions)) {
        mutex.lock();
        RECON_LOG(PIPELINE) << "读取特征失败。";
        mutex.unlock();
        return false;
    }
    
    progress = 0.2f;
    matches_provider = std::make_shared<Matches_Provider>();
    if (!matches_provider->load(sfm_data, "products/matches/matches.f.bin")) {
        mutex.lock();
        RECON_LOG(PIPELINE) << "匹配读取失败。";
        mutex.unlock();
        return false;
    }
    SequentialSfMReconstructionEngine sfm_engine(sfm_data, "products/sfm", "products/sfm/report.html");
    sfm_engine.SetFeaturesProvider(features_provider.get());
    sfm_engine.SetMatchesProvider(matches_provider.get());
    sfm_engine.Set_Intrinsics_Refinement_Type(intrinsic_refinement_options);
    sfm_engine.SetUnknownCameraType(camera_model);
    sfm_engine.Set_Use_Motion_Prior(false);
    sfm_engine.SetTriangulationMethod(triangulation_method);
    sfm_engine.SetResectionMethod(resection_method);
    
    progress = 0.5f;
    if (!sfm_engine.Process()) {
        mutex.lock();
        RECON_LOG(PIPELINE) << "SfM 引擎处理失败。";
        mutex.unlock();
        return false;
    }
    progress = 0.9f;
    mutex.lock();
    RECON_LOG(PIPELINE) << "SfM 引擎处理完毕。正在导出数据...";
    mutex.unlock();
    
    Save(sfm_engine.Get_SfM_Data(), "products/sfm/sfm_engine_data.bin", ESfM_Data(ALL));
    Save(sfm_engine.Get_SfM_Data(), "products/sfm/clouds_and_poses.ply", ESfM_Data(ALL));
    
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
    
    const auto intrinsic_refinement_options = cameras::Intrinsic_Parameter_Type::ADJUST_FOCAL_LENGTH |
        cameras::Intrinsic_Parameter_Type::ADJUST_PRINCIPAL_POINT |
        cameras::Intrinsic_Parameter_Type::ADJUST_DISTORTION;
    
    GlobalSfMReconstructionEngine_RelativeMotions sfm_engine(sfm_data, "products/sfm", "products/sfm/global_report.html");
    sfm_engine.SetFeaturesProvider(features_provider.get());
    sfm_engine.SetMatchesProvider(matches_provider.get());
    sfm_engine.Set_Intrinsics_Refinement_Type(intrinsic_refinement_options);
    sfm_engine.Set_Use_Motion_Prior(false);
    sfm_engine.SetRotationAveragingMethod(ERotationAveragingMethod(ROTATION_AVERAGING_L2));
    sfm_engine.SetTranslationAveragingMethod(ETranslationAveragingMethod(TRANSLATION_AVERAGING_SOFTL1));
    
    progress = 0.5f;
    if (!sfm_engine.Process()) {
        mutex.lock();
        RECON_LOG(PIPELINE) << "SfM 全局引擎处理失败。";
        mutex.unlock();
        return false;
    }
    
    progress = 0.9f;
    mutex.lock();
    RECON_LOG(PIPELINE) << "SfM 引擎处理完毕。正在导出数据...";
    mutex.unlock();
    
    Save(sfm_engine.Get_SfM_Data(), "products/sfm/sfm_engine_data.bin", ESfM_Data(ALL));
    Save(sfm_engine.Get_SfM_Data(), "products/sfm/clouds_and_poses.ply", ESfM_Data(ALL));
    
    progress = 1.0f;
    mutex.lock();
    RECON_LOG(PIPELINE) << "全局 SfM 结束。正在更新 SfM 数据...";
    sfm_data = sfm_engine.Get_SfM_Data();
    mutex.unlock();
    return true;
}

auto Pipeline::colorize(PipelineState state) -> bool {
    progress = 0.0f;
    this->state = state;
    mutex.lock();
    RECON_LOG(PIPELINE) << "开始进行上色处理。";
    mutex.unlock();
    
    std::vector<Vec3> points, tracks_color, cam_position;
    if (!ColorizeTracks(sfm_data, points, tracks_color)) {
        mutex.lock();
        RECON_LOG(PIPELINE) << "上色处理失败：无法为轨迹上色。";
        mutex.unlock();
        return false;
    }
    progress = 0.5f;
    for (const auto &view : sfm_data.GetViews()) {
        if (sfm_data.IsPoseAndIntrinsicDefined(view.second.get())) {
            const Pose3 pose = sfm_data.GetPoseOrDie(view.second.get());
            cam_position.push_back(pose.center());
        }
    }
    progress = 0.8f;
    std::string path = "products/sfm/global_recon_colorized.ply";
    if (state == PipelineState::COLORIZED_ROBUST_TRIANGULATION) {
        path = "products/sfm/robust_recon_colorized.ply";
    }
    if (!export_to_ply(path, points, cam_position, tracks_color)) {
        mutex.lock();
        RECON_LOG(PIPELINE) << "模型导出失败。跳过步骤。";
        mutex.unlock();
    }
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
    
    const auto max_reprojection_error = 4.0;
    const auto triangulation_method = ETriangulationMethod::DEFAULT;
    std::unique_ptr<Regions> regions(new SIFT_Regions());
    auto regions_provider = std::make_shared<Regions_Provider>();
    if (!regions_provider->load(sfm_data, "products/features", regions, nullptr)) {
        RECON_LOG(PIPELINE) << "区间错误。";
        return false;
    }
    
    Pair_Set pairs;
    PairWiseMatches matches;
    if (!Load(matches, "products/matches/matches.f.bin")) {
        mutex.lock();
        RECON_LOG(PIPELINE) << "无法加载匹配文件。";
        mutex.unlock();
        return false;
    }
    pairs = getPairs(matches);
    const std::set<IndexT> valid_views_idx = Get_Valid_Views(sfm_data);
    pairs = Pair_filter(pairs, valid_views_idx);
    
    progress = 0.5f;
    SfM_Data_Structure_Estimation_From_Known_Poses structure_estimator(max_reprojection_error);
    structure_estimator.run(sfm_data, pairs, regions_provider, triangulation_method);
    
    regions.reset();
    RemoveOutliers_AngleError(sfm_data, 2.0);
    progress = 0.7f;
    mutex.lock();
    RECON_LOG(PIPELINE) << "地标数量：" << sfm_data.GetLandmarks().size();
    RECON_LOG(PIPELINE) << "处理结束。正在保存...";
    mutex.unlock();
    
    Save(sfm_data, "products/sfm/robust.ply", ESfM_Data(ALL));
    Save(sfm_data, "products/sfm/robust.bin", ESfM_Data(ALL));
    
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
    
    MVS::Interface scene;
    const auto num_views = sfm_data.GetViews().size();
    std::map<IndexT, uint32_t> map_intrinsic, map_view;
    auto i = 0;
    for (const auto &intrinsic : sfm_data.GetIntrinsics()) {
        progress = ((float) i / sfm_data.GetIntrinsics().size()) * 0.33f;
        i++;
        if (isPinhole(intrinsic.second->getType())) {
            const Pinhole_Intrinsic *cam = (const Pinhole_Intrinsic *) intrinsic.second.get();
            if (map_intrinsic.count(intrinsic.first) == 0) {
                map_intrinsic[intrinsic.first] = (int) scene.platforms.size();
            }
            MVS::Interface::Platform platform;
            MVS::Interface::Platform::Camera camera;
            camera.width = cam->w();
            camera.height = cam->h();
            camera.K = cam->K();
            camera.R = Mat3::Identity();
            camera.C = Vec3::Zero();
            platform.cameras.push_back(camera);
            scene.platforms.push_back(platform);
        } else {
            mutex.lock();
            RECON_LOG(PIPELINE) << "摄像机 #" << intrinsic.first << " 不是针孔摄像机；跳过。";
            mutex.unlock();
        }
    }
    
    scene.images.reserve(num_views);
    auto num_poses = 0;
    i = 0;
    for (const auto &view : sfm_data.GetViews()) {
        progress = ((float) i / sfm_data.GetIntrinsics().size()) * 0.33f + 0.33f;
        i++;
        
        if (sfm_data.IsPoseAndIntrinsicDefined(view.second.get())) {
            map_view[view.first] = (int) scene.images.size();
            
            MVS::Interface::Image image;
            std::filesystem::path image_path = view.second->s_Img_path;
            std::filesystem::path image_file_name = image_path.filename();
            if (image_file_name.extension() == ".jpeg") {
                image_file_name.replace_extension(".jpg");
            }
            image.name = "products/mvs/images/" + image_file_name.string();
            image.platformID = map_intrinsic.at(view.second->id_intrinsic);
            auto &platform = scene.platforms[image.platformID];
            image.cameraID = 0;
            
            // Just copy all the photos to destination, since we don't have distortion enabled
            std::filesystem::path dest = image.name;
            if (std::filesystem::exists(dest)) {
                std::filesystem::remove(dest);
            }
            std::filesystem::copy(image_path, dest);
            
            MVS::Interface::Platform::Pose pose;
            image.poseID = (int) platform.poses.size();
            const Pose3 pose_mvg(sfm_data.GetPoseOrDie(view.second.get()));
            pose.R = pose_mvg.rotation();
            pose.C = pose_mvg.center();
            platform.poses.push_back(pose);
            num_poses++;
            scene.images.emplace_back(image);
        } else {
            mutex.lock();
            RECON_LOG(PIPELINE) << "无法读取对应相机坐标：#" << (i - 1);
            mutex.unlock();
        }
    }
    
    i = 0;
    scene.vertices.reserve(sfm_data.GetLandmarks().size());
    for (const auto &vertex : sfm_data.GetLandmarks()) {
        float sub_progress = (float) i / sfm_data.GetLandmarks().size();
        i++;
        progress = sub_progress * 0.33f + 0.66f;
        const auto &landmark = vertex.second;
        
        MVS::Interface::Vertex vert;
        MVS::Interface::Vertex::ViewArr &views = vert.views;
        for (const auto &observation : landmark.obs) {
            const auto it(map_view.find(observation.first));
            if (it != map_view.end()) {
                MVS::Interface::Vertex::View view;
                view.imageID = it->second;
                view.confidence = 0;
                views.push_back(view);
            }
        }
        if (views.size() < 2) {
            continue;
        }
        std::sort(views.begin(), views.end(), [] (const auto &v0, const auto &v1) {
            return v0.imageID < v1.imageID;
        });
        vert.X = landmark.X.cast<float>();
        scene.vertices.push_back(vert);
    }
    if (!MVS::ARCHIVE::SerializeSave(scene, "products/mvs/scene.mvs")) {
        mutex.lock();
        RECON_LOG(PIPELINE) << "MVS 场景保存失败。";
        mutex.unlock();
        return false;
    }
    
    progress = 1.0f;
    mutex.lock();
    RECON_LOG(PIPELINE) << "格式转换完成。平台数量：" << scene.platforms.size();
    mutex.unlock();
    
    open_mvs.init();
    return true;
}

auto Pipeline::mvs_procedures() -> bool {
    state = PipelineState::DENSIFY_PC;
    if (!open_mvs.density_point_cloud(progress)) {
        return false;
    }

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
                }
                mutex.unlock();
                if (pipeline.state != PipelineState::FINISHED_ERR &&
                    pipeline.state != PipelineState::FINISHED_SUCCESS) {
                    ImGui::ProgressBar(pipeline.progress);
                }
                break;
        }
        
        if (ImGui::Button("导出 SfM 文件")) {
            pipeline.save_sfm("sfm_data.json");
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
