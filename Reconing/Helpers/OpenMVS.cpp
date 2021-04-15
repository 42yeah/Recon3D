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
    
    
    
    mutex.lock();
    RECON_LOG(OMVS) << "点云稠密完成。";
    mutex.unlock();
    progress = 1.0f;
    return true;
}
