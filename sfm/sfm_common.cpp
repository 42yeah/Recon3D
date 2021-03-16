//
//  sfm_common.cpp
//  sfm
//
//  Created by apple on 15/03/2021.
//

#include "sfm_common.hpp"


void keypoints_to_points(IN const KeyPoints &kps, OUT Points2f &ps) {
    ps.clear();
    for (const cv::KeyPoint &kp : kps) {
        ps.push_back(kp.pt);
    }
}
