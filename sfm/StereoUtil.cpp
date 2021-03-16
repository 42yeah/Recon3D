//
//  StereoUtil.cpp
//  sfm
//
//  Created by apple on 15/03/2021.
//

#include "StereoUtil.hpp"
#include <opencv2/calib3d.hpp>


void StereoUtil::init() {
    
}

int StereoUtil::find_homography_inliers(const Features &left, const Features &right, const Matches &matches) { 
    Features aligned_left, aligned_right;
    
    get_aligned_points_from_match(left, right, matches, aligned_left, aligned_right);
    
    cv::Mat inlier_mask, homogrpahy;
    
    if (matches.size() >= 4) {
        homogrpahy = cv::findHomography(aligned_left.points, aligned_right.points, cv::RANSAC, RANSAC_THRESHOLD, inlier_mask);
    }
    if (matches.size() < 4 || homogrpahy.empty()) {
        return 0;
    }
    return cv::countNonZero(inlier_mask);
}

void StereoUtil::get_aligned_points_from_match(const Features &left, const Features &right, const Matches &matches, Features &aligned_left, Features &aligned_right, std::vector<int> &left_back_ref, std::vector<int> &right_back_ref) { 
    aligned_left.keypoints.clear();
    aligned_right.keypoints.clear();
    aligned_left.descriptors = cv::Mat();
    aligned_right.descriptors = cv::Mat();
    
    // For each matches...
    for (int i = 0; i < matches.size(); i++) {
        // Find matching keypoint from left, and push it into aligned_left
        aligned_left.keypoints.push_back(left.keypoints[matches[i].queryIdx]);
        // Find matching descriptor from left, and push it into aligned_left
        aligned_left.descriptors.push_back(left.descriptors.row(matches[i].queryIdx));
        // ... And do the same for the right
        aligned_right.keypoints.push_back(right.keypoints[matches[i].trainIdx]);
        aligned_right.descriptors.push_back(right.descriptors.row(matches[i].trainIdx));
        
        left_back_ref.push_back(matches[i].queryIdx);
        right_back_ref.push_back(matches[i].trainIdx);
    }
    
    keypoints_to_points(aligned_left.keypoints, aligned_left.points);
    keypoints_to_points(aligned_right.keypoints, aligned_right.points);
    
}

void StereoUtil::get_aligned_points_from_match(const Features &left, const Features &right, const Matches &matches, Features &aligned_left, Features &aligned_right) { 
    std::vector<int> trash_left, trash_right;
    get_aligned_points_from_match(left, right, matches, aligned_left, aligned_right, trash_left, trash_right);
}

bool StereoUtil::find_camera_matrices_from_match(const Intrinsics &intrinsics, const Matches &matches, const Features &left, const Features &right, Matches &pruned_matches, cv::Matx34f &p_left, cv::Matx34f &p_right) {
    if (intrinsics.k.empty()) {
        LOG("Must initialize K!");
        return false;
    }
    
    float focal = intrinsics.k.at<float>(0, 0);
    cv::Point2d pp(intrinsics.k.at<float>(0, 2), intrinsics.k.at<float>(1, 2));
    
    Features aligned_left, aligned_right;
    get_aligned_points_from_match(left, right, matches, aligned_left, aligned_right);
    
    // Essential matrix, rotation matrix, translation vector
    cv::Mat E, R, t;
    cv::Mat mask;
    
    E = cv::findEssentialMat(aligned_left.points,
                             aligned_right.points,
                             focal,
                             pp,
                             cv::RANSAC,
                             0.999f,
                             1.0f,
                             mask);
    cv::recoverPose(E, aligned_left.points, aligned_right.points, R, t, focal, pp, mask);
    
    p_left = cv::Matx34f::eye();
    p_right = cv::Matx34f(R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),  t.at<double>(0),
                          R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),t.at<double>(1),
                          R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2));
    
    pruned_matches.clear();
    
    for (int i = 0; i < mask.rows; i++) {
        if (mask.at<unsigned char>(i)) {
            pruned_matches.push_back(matches[i]);
        }
    }
    return true;
}


bool StereoUtil::triangulate_views(const Intrinsics &intrinsics, const ImagePair &pair, const Matches &matches, const Features &left_features, const Features &right_features, const cv::Matx34f &p_left, const cv::Matx34f &p_right, PointCloud &cloud) {
    std::vector<int> back_left, back_right;
    Features aligned_left, aligned_right;
    
    get_aligned_points_from_match(left_features, right_features, matches, aligned_left, aligned_right, back_left, back_right);
    
    cv::Mat normalized_left_pts, normalized_right_pts;
    cv::undistortPoints(aligned_left.points, normalized_left_pts, intrinsics.k, cv::Mat());
    cv::undistortPoints(aligned_right.points, normalized_right_pts, intrinsics.k, cv::Mat());
    
    cv::Mat points_3d_homogeneous;
    cv::triangulatePoints(p_left, p_right, normalized_left_pts, normalized_right_pts, points_3d_homogeneous);
    
    cv::Mat points_3d;
    cv::convertPointsFromHomogeneous(points_3d_homogeneous.t(), points_3d);
    
    cv::Mat rvec_left;
    cv::Rodrigues(p_left.get_minor<3, 3>(0, 0), rvec_left);
    cv::Mat tvec_left(p_left.get_minor<3, 1>(0, 3).t());
    
    Points2f projected_on_left(aligned_left.points.size());
    cv::projectPoints(points_3d, rvec_left, tvec_left, intrinsics.k, cv::Mat(), projected_on_left);
    
    cv::Mat rvec_right;
    cv::Rodrigues(p_right.get_minor<3, 3>(0, 0), rvec_right);
    cv::Mat tvec_right(p_right.get_minor<3, 1>(0, 3).t());
    
    Points2f projected_on_right(aligned_right.points.size());
    cv::projectPoints(points_3d, rvec_right, tvec_right, intrinsics.k, cv::Mat(), projected_on_right);
    
    for (int i = 0; i < points_3d.rows; i++) {
        if (cv::norm(projected_on_left[i] - aligned_left.points[i]) > MIN_PROJECTION_ERROR ||
            cv::norm(projected_on_right[i] - aligned_right.points[i]) > MIN_PROJECTION_ERROR) {
            continue;
        }
        
        Point3DInMap p;
        p.point = cv::Point3f(points_3d.at<float>(i, 0),
                              points_3d.at<float>(i, 1),
                              points_3d.at<float>(i, 2));
        p.originating_views[pair.left] = back_left[i];
        p.originating_views[pair.right] = back_right[i];
        
        cloud.push_back(p);
    }
    
    return true;
}

