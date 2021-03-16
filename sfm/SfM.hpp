//
//  SfM.hpp
//  sfm
//
//  Created by apple on 15/03/2021.
//

#ifndef SfM_hpp
#define SfM_hpp

#include <vector>
#include <map>
#include <iostream>
#include <set>
#include "sfm_common.hpp"
#include "FeatureUtil.hpp"
#include "StereoUtil.hpp"

#define MIN_POINT_COUNT_FOR_HOMOGRAPHY 100
#define POSE_INLIER_MINIMAL_RATIO 0.5f


enum Result {
    OK,
    ERR
};

class SfM {
    typedef std::vector<std::vector<Matches> > MatchMat;
    
    typedef std::map<int, Image2D3DMatch> Image2D3DMatches;
    
public:
    SfM() {}
    
    ~SfM() = default;
    
    bool set_image_dir(std::string path);
    
    Result run_sfm();
    
    /// Export the point cloud to .ply (if there is one).
    /// @param path path to save the ply
    void export_to_ply(std::string path);
    
private:
    void extract_features();
    
    void create_feature_match_matrix();
    
    void find_baseline_triangulation();
    
    void adjust_current_bundle();
    
    std::map<float, ImagePair> sort_views_for_baseline();
    
    void add_more_views_to_reconstruction();
    
    Image2D3DMatches find_2d_3d_matches();
    
    void merge_new_pointclouds(const PointCloud &cloud);
    
    // Utilities
    FeatureUtil feature_util;
    StereoUtil stereo_util;
    
    // Variables
    std::vector<cv::Mat> images;
    Intrinsics intrinsics;
    std::vector<cv::Matx34f> camera_poses;
    std::vector<Features> features;
    MatchMat feature_match_mat;
    PointCloud reconstruction_cloud;
    std::set<int> done_views;
    std::set<int> good_views;
};

#endif /* SfM_hpp */
