//
//  OpenMVS.hpp
//  Reconing
//
//  Created by apple on 14/04/2021.
//

#ifndef OpenMVS_hpp
#define OpenMVS_hpp

// O P E N M V S ////////////////////////////
enum ARCHIVE_TYPE {
    ARCHIVE_MVS = -1,
    ARCHIVE_TEXT = 0,
    ARCHIVE_BINARY,
    ARCHIVE_BINARY_ZIP,
    ARCHIVE_LAST
};

#include <vector>
#include <mutex>

#define OMVS "多视图立体重建"

class OpenMVS {
public:
    OpenMVS() {}
    
    auto init() -> bool;
    
    // P I P E L I N E //////////////////////////
    auto density_point_cloud(float &progress) -> bool;

    auto reconstruct_mesh(float &progress) -> bool;

private:
    std::mutex mutex;
};

#endif /* OpenMVS_hpp */
