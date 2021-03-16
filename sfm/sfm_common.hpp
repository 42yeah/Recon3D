//
//  sfm_common.hpp
//  sfm
//
//  Created by apple on 15/03/2021.
//

#ifndef sfm_common_hpp
#define sfm_common_hpp

#include <iostream>
#include <vector>
#include <map>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#define LOG_LEVEL 2
#define IN
#define OUT

#if LOG_LEVEL == 0
#define LOG(...)
#elif LOG_LEVEL == 1
#define LOG(fmt, ...) printf(fmt "\n", ##__VA_ARGS__)
#elif LOG_LEVEL == 2
#define LOG(fmt, ...) printf("%s:%d: " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__)
#endif


struct Intrinsics {
    cv::Mat k;
    cv::Mat k_inv;
    cv::Mat distortion;
};

struct ImagePair {
    int left, right;
};

struct Point3DInMap {
    cv::Point3f point;
    std::map<int, int> originating_views;
};

typedef std::vector<cv::KeyPoint> KeyPoints;
typedef std::vector<cv::Point2f> Points2f;
typedef std::vector<cv::Point3f> Points3f;
typedef std::vector<cv::DMatch> Matches;
typedef std::vector<Point3DInMap> PointCloud;

struct Image2D3DMatch {
    Points2f points_2d;
    Points3f points_3d;
};

struct Features {
    KeyPoints keypoints;
    Points2f points;
    cv::Mat descriptors;
};


void keypoints_to_points(IN const KeyPoints &kps, OUT Points2f &ps);

#endif /* sfm_common_hpp */
