//
//  StereoUtil.hpp
//  sfm
//
//  Created by apple on 15/03/2021.
//

#ifndef StereoUtil_hpp
#define StereoUtil_hpp

#include "sfm_common.hpp"
#define RANSAC_THRESHOLD 10.0f
#define MIN_PROJECTION_ERROR 10.0f


class StereoUtil {
public:
    StereoUtil() {}

    void init();
    
    
    /// Find the amount of inlier points in a homography between two views.
    /// @param left left image features.
    /// @param right right image features.
    /// @param matches matches between the features.
    int find_homography_inliers(const Features &left, const Features &right, const Matches &matches);
    
    
    /// Find camera matrices (3x4) from stereo point matching.
    /// @param intrinsics camera intrinsics.
    /// @param matches matches over images.
    /// @param left features from left image.
    /// @param right features from right image.
    /// @param pruned_matches matches after pruning using essential matrix.
    /// @param p_left left image matrix.
    /// @param p_right right image matrix.
    bool find_camera_matrices_from_match(IN const Intrinsics &intrinsics,
                                         IN const Matches &matches,
                                         IN const Features &left,
                                         IN const Features &right,
                                         OUT Matches &pruned_matches,
                                         OUT cv::Matx34f &p_left,
                                         OUT cv::Matx34f &p_right);
    
    
    /// Triangulate (recover 3D positions) from point matching.
    /// @param intrinsics camera intrinsics.
    /// @param pair indices of left/right views.
    /// @param matches matches over image pair.
    /// @param left_features left image features.
    /// @param right_features right image features.
    /// @param p_left left camera mat.
    /// @param p_right right camera mat.
    /// @param cloud point cloud with image associations.
    /// @return true on success.
    bool triangulate_views(IN const Intrinsics &intrinsics,
                           IN const ImagePair &pair,
                           IN const Matches &matches,
                           IN const Features &left_features,
                           IN const Features &right_features,
                           IN const cv::Matx34f &p_left,
                           IN const cv::Matx34f &p_right,
                           OUT PointCloud &cloud);
    
    
    /// Get the features for left and right images after keeping the only matched features and aligning them.
    /// @param left left image features.
    /// @param right right image features.
    /// @param matches matches over features.
    /// @param aligned_left aligned left features.
    /// @param aligned_right aligned right features.
    /// @param left_back_ref back reference from aligned index to original index.
    /// @param right_back_ref back reference from aligned index to original index.
    void get_aligned_points_from_match(IN const Features &left,
                                       IN const Features &right,
                                       IN const Matches &matches,
                                       OUT Features &aligned_left,
                                       OUT Features &aligned_right,
                                       OUT std::vector<int> &left_back_ref,
                                       OUT std::vector<int> &right_back_ref);
    
    
    /// Just like the function above, but dumping the refs.
    /// @param left left image features.
    /// @param right right image features.
    /// @param matches matches over features.
    /// @param aligned_left aligned left features.
    /// @param aligned_right aligned right features.
    void get_aligned_points_from_match(IN const Features &left,
                                       IN const Features &right,
                                       IN const Matches &matches,
                                       OUT Features &aligned_left,
                                       OUT Features &aligned_right);
};

#endif /* StereoUtil_hpp */
