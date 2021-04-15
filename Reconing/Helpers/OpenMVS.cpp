//
//  OpenMVS.cpp
//  Reconing
//
//  Created by apple on 14/04/2021.
//

#include "OpenMVS.hpp"
#define _USE_EIGEN
#define _USE_OPENCV
#include <Eigen/Geometry>
#include <MVS/Common.h>
#include <MVS/Scene.h>
#include <mutex>
#include <Common/Types.h>
#include "common.hpp"

using namespace MVS;


auto OpenMVS::init() -> bool {
    mutex.lock();
    RECON_LOG(OMVS) << "OpenMVS 已经初始化。";
    mutex.unlock();
    return true;
}

auto OpenMVS::density_point_cloud(float &progress) -> bool {
    const auto dense_config_file = "products/mvs/densify.ini";
    const auto resolution_level = 1;
    progress = 0.0f;
    mutex.lock();
    RECON_LOG(OMVS) << "正在稠密点云...";
    mutex.unlock();
    
    WORKING_FOLDER = ".";
    INIT_WORKING_FOLDER;
    
    OPTDENSE::init();
    OPTDENSE::update();
    OPTDENSE::nResolutionLevel = resolution_level;
    OPTDENSE::nMaxResolution = 3200;
    OPTDENSE::nMinResolution = 640;
    OPTDENSE::nNumViews = 5;
    OPTDENSE::nMinViewsFuse = 3;
    OPTDENSE::nEstimateColors = 2;
    OPTDENSE::nEstimateNormals = 2;
    OPTDENSE::oConfig.Save(dense_config_file);
    
    Process::setCurrentProcessPriority(Process::Priority::BELOWNORMAL);
    Util::Init();
    
    mutex.lock();
    RECON_LOG(OMVS) << "点云稠密正在开始。使用所有线程。";
    mutex.unlock();
    Scene scene(0);
    progress = 0.2f;
    if (!scene.Load("products/mvs/scene.mvs")) {
        mutex.lock();
        RECON_LOG(OMVS) << "找不到场景文件：products/mvs/scene.mvs。";
        mutex.unlock();
        return false;
    }
    if (scene.pointcloud.IsEmpty()) {
        mutex.lock();
        RECON_LOG(OMVS) << "场景内不存在点云。";
        mutex.unlock();
        return false;
    }
    TD_TIMER_START();
    progress = 0.5f;
    if (!scene.DenseReconstruction(0)) {
        mutex.lock();
        RECON_LOG(OMVS) << "深度图测得 " << TD_TIMER_GET_FMT().c_str();
        mutex.unlock();
    } else {
        RECON_LOG(OMVS) << "稠密点云报告：一共 " << scene.pointcloud.GetSize() << " 个点";
    }
    
    scene.Save("products/mvs/dense.mvs", ARCHIVE_BINARY_ZIP);
    scene.pointcloud.Save("products/mvs/dense.ply");
    
    mutex.lock();
    RECON_LOG(OMVS) << "点云稠密完成。";
    mutex.unlock();
    progress = 1.0f;
    return true;
}
