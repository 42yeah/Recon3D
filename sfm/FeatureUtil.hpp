//
//  FeatureUtil.hpp
//  sfm
//
//  Created by apple on 15/03/2021.
//

#ifndef FeatureUtil_hpp
#define FeatureUtil_hpp

#include <opencv2/features2d.hpp>
#include "sfm_common.hpp"


class FeatureUtil {
public:
    FeatureUtil() {}
    
    void init();
    
    Features extract_features(const cv::Mat &image);
    
    Matches match_features(const Features &left, const Features &right);
    
private:
    cv::Ptr<cv::Feature2D> detector;
    cv::Ptr<cv::DescriptorMatcher> matcher;
};

#endif /* FeatureUtil_hpp */
