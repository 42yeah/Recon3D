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

using namespace openMVG::exif;
using namespace openMVG::geodesy;
using namespace openMVG::features;


using namespace PipelineNS;

std::mutex mutex;

auto run_pipeline(Pipeline *pipeline_ptr) -> void {
    auto &pipeline = *pipeline_ptr;
    
    pipeline.run();
}

auto Pipeline::run() -> bool {
    if (!std::filesystem::exists("products")) {
        std::filesystem::create_directory("products");
    }
    if (!intrinsics_analysis()) {
        mutex.lock();
        LOG(PIPELINE) << "相机内部参数提取错误。";
        mutex.unlock();
        state = PipelineState::FINISHED_ERR;
        return false;
    }
    if (!feature_detection()) {
        mutex.lock();
        LOG(PIPELINE) << "图片特征提取错误。";
        mutex.unlock();
        state = PipelineState::FINISHED_ERR;
        return false;
    }
    return true;
}

auto Pipeline::intrinsics_analysis() -> bool {
    progress = 0.0f;
    state = PipelineState::INTRINSICS_ANALYSIS;
    LOG(PIPELINE) << "相机内部参数提取开始。";
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
        auto region = image_describer->Describe(image, mask);
        
        auto pov = path_of_view(*view);
        if (!std::filesystem::exists(pov)) {
            std::filesystem::create_directory(pov);
        }

        if (region && !image_describer->Save(region.get(),
                                             pov / "features",
                                             pov / "descriptors")) {
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

auto Pipeline::path_of_view(const openMVG::sfm::View &view) -> std::filesystem::path { 
    return std::filesystem::path("products") / std::to_string(view.id_view);
}




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

