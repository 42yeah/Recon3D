//
//  OpenMVS.hpp
//  Reconing
//
//  Created by apple on 14/04/2021.
//

#ifndef OpenMVS_hpp
#define OpenMVS_hpp

// O P E N M V S ////////////////////////////
#include <vector>
#define _USE_EIGEN
#include <Eigen/Geometry>
#include <MVS/Interface.h>


class OpenMVS {
public:
    OpenMVS() {}

    OpenMVS(MVS::Interface scene) : scene(scene) {}

    MVS::Interface scene;
};

#endif /* OpenMVS_hpp */
