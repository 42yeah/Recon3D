//
//  FeatureUtil.cpp
//  sfm
//
//  Created by apple on 15/03/2021.
//

#include "FeatureUtil.hpp"
#include <opencv2/imgcodecs.hpp>


Features FeatureUtil::extract_features(const cv::Mat &image) {
    Features features;
    detector->detectAndCompute(image, cv::noArray(), features.keypoints, features.descriptors);
    keypoints_to_points(features.keypoints, features.points);
    return features;
}

void FeatureUtil::init() { 
    detector = cv::ORB::create(5000);
    matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
}

Matches FeatureUtil::match_features(const Features &left, const Features &right) { 
    std::vector<Matches> initial_matching;
    
    matcher->knnMatch(left.descriptors, right.descriptors, initial_matching, 2);
    Matches pruned_matches;
    for (int i = 0; i < initial_matching.size(); i++) {
        if (initial_matching[i][0].distance < 0.8f * initial_matching[i][1].distance) {
            pruned_matches.push_back(initial_matching[i][0]);
        }
    }
    return pruned_matches;
}


