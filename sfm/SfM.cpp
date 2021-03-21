//
//  SfM.cpp
//  sfm
//
//  Created by apple on 15/03/2021.
//

#include "SfM.hpp"
#include <filesystem>
#include <thread>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>


Result SfM::run_sfm() {
    if (images.empty()) {
        SFM_LOG("There is no image to work on.");
        return ERR;
    }
    
    // Initialize feature matcher
    feature_util.init();
    stereo_util.init();
    
    // Initialize intrinsics
    intrinsics.k = (cv::Mat_<float>(3, 3) << 2500, 0, images[0].cols / 2,
                                             0, 2500, images[0].rows / 2,
                                             0, 0, 1);
    intrinsics.k_inv = intrinsics.k.inv();
    intrinsics.distortion = cv::Mat_<float>::zeros(1, 4);
    
    camera_poses.resize(images.size());
    
    extract_features();
    create_feature_match_matrix();
    find_baseline_triangulation();
    add_more_views_to_reconstruction();
    
    return OK;
}

bool SfM::set_image_dir(std::string path) {
    SFM_LOG("Setting image directory as %s", path.c_str());
    try {
        for (const auto &entry : std::filesystem::directory_iterator(path)) {
            cv::Mat img = cv::imread(entry.path());
            if (img.rows == 0 || img.cols == 0) {
                SFM_LOG("WARNING! Invalid image: %s. Skipping...", entry.path().c_str());
                continue;
            }
            images.push_back(std::move(img));
        }
    } catch (std::exception &e) {
        SFM_LOG("Failed to load image directory.");
        return false;
    }
    return true;
}

void SfM::extract_features() { 
    SFM_LOG("Extracting features...");
    
    features.resize(images.size());
    for (int i = 0; i < images.size(); i++) {
        features[i] = feature_util.extract_features(images[i]);
        SFM_LOG("Image %d has %ld keypoints", i, features[i].points.size());
    }
}

void SfM::create_feature_match_matrix() {
    SFM_LOG("Creating feature match matrix...");
    
    int num_images = (int) images.size();
    feature_match_mat.resize(num_images, std::vector<Matches>(num_images));
    
    std::vector<ImagePair> pairs;
    for (int i = 0; i < num_images; i++) {
        for (int j = i + 1; j < num_images; j++) {
            pairs.push_back({ i, j });
        }
    }
    
    std::vector<std::thread> threads;
    int num_threads = std::thread::hardware_concurrency() - 1;
    int num_pairs_for_thread = (num_threads > pairs.size()) ? 1 :
        (int) ceilf((float) pairs.size() / num_threads);
    
    std::mutex write_mutex;
    SFM_LOG("Launching %d threads with %d pairs per thread...", num_threads, num_pairs_for_thread);
    for (int thread_id = 0; thread_id < std::min(num_threads, (int) pairs.size()); thread_id++) {
        threads.push_back(std::thread([&, thread_id] () {
            const int starting_pair = num_pairs_for_thread * thread_id;
            for (int j = 0; j < num_pairs_for_thread; j++) {
                const int pair_id = starting_pair + j;
                if (pair_id >= pairs.size()) {
                    break;
                }
                const ImagePair &pair = pairs[pair_id];
                feature_match_mat[pair.left][pair.right] = feature_util.match_features(features[pair.left], features[pair.right]);
                
                write_mutex.lock();
                SFM_LOG("Thread %d -> pair %d: %ld matches", thread_id, pair_id, feature_match_mat[pair.left][pair.right].size());
                write_mutex.unlock();
            }
        }));
    }
    
    for (auto &t : threads) {
        t.join();
    }
}

void SfM::find_baseline_triangulation() { 
    SFM_LOG("Finding baseline triangulation...");

    std::map<float, ImagePair> homography_inliers = sort_views_for_baseline();
    
    PointCloud cloud;
    
    SFM_LOG("Trying views in triangulation...");
    for (auto &pair : homography_inliers) {
        SFM_LOG("Trying (%d, %d) ratio: %f", pair.second.left, pair.second.right, pair.first);
        
        cv::Matx34f p_left = cv::Matx34f::eye();
        cv::Matx34f p_right = cv::Matx34f::eye();
        int left = pair.second.left;
        int right = pair.second.right;

        Matches pruned_matches;
        
        SFM_LOG("Finding camera matrices...");
        bool success = stereo_util.find_camera_matrices_from_match(intrinsics, feature_match_mat[left][right], features[left], features[right], pruned_matches, p_left, p_right);
        if (!success) {
            SFM_LOG("WARNING! Camera matrix recovery failed.");
        }
        
        float pose_inlier_ratio = (float) pruned_matches.size() / feature_match_mat[left][right].size();
        SFM_LOG("Pruned inlier ratio of (%d, %d): %f", left, right, pose_inlier_ratio);
        if (pose_inlier_ratio < POSE_INLIERS_MINIMAL_RATIO) {
            SFM_LOG("Insufficient pose inliers. Skipping.");
            continue;
        }
        
#if LOG_LEVEL >= 2
        // Visual debug out image
        cv::Mat out_img;
        cv::drawMatches(images[left], features[left].keypoints, images[right], features[right].keypoints, pruned_matches, out_img);
        cv::imshow("match", out_img);
        cv::imwrite(std::to_string(left) + "_" + std::to_string(right) + ".jpg", out_img);
        cv::waitKey(0);
#endif
        
        feature_match_mat[left][right] = pruned_matches;
        
        SFM_LOG("Triangulating from stereo views (%d, %d)...", left, right);
        success = stereo_util.triangulate_views(intrinsics, pair.second, feature_match_mat[left][right], features[left], features[right], p_left, p_right, cloud);
        if (!success) {
            SFM_LOG("Could not triangulate: (%d, %d).", left, right);
            continue;
        }

        reconstruction_cloud.insert(reconstruction_cloud.end(), cloud.begin(), cloud.end());
        camera_poses[left] = p_left;
        camera_poses[right] = p_right;
        done_views.insert(left);
        done_views.insert(right);
        good_views.insert(left);
        good_views.insert(right);
        
        break;
    }
}

std::map<float, ImagePair> SfM::sort_views_for_baseline() {
    SFM_LOG("Finding views homography inliers...");
    
    std::map<float, ImagePair> matches_sizes;
    int num_images = (int) images.size();
    
    for (int i = 0; i < num_images - 1; i++) {
        for (int j = i + 1; j < num_images; j++) {
            if (feature_match_mat[i][j].size() < MIN_POINT_COUNT_FOR_HOMOGRAPHY) {
                // There is no enough points for matching
                matches_sizes[1.0f] = { i, j };
                continue;
            }
            
            const int num_inliers = stereo_util.find_homography_inliers(features[i], features[j], feature_match_mat[i][j]);
            float inliers_ratio = (float) num_inliers / feature_match_mat[i][j].size();
            matches_sizes[inliers_ratio] = { i, j };
            SFM_LOG("Homography inlier ratio: %d %d %f", i, j, inliers_ratio);
        }
    }
    return matches_sizes;
}

void SfM::export_to_ply(std::string path) { 
    SFM_LOG("Saving point cloud to %s", path.c_str());
    
    std::ofstream ofs(path);
    ofs << "ply" << std::endl
        << "format ascii 1.0" << std::endl
        << "element vertex " << reconstruction_cloud.size() << std::endl
        << "property float x" << std::endl
        << "property float y" << std::endl
        << "property float z" << std::endl
        << "property uchar red" << std::endl
        << "property uchar green" << std::endl
        << "property uchar blue" << std::endl
        << "end_header" << std::endl;
        
    for (const Point3DInMap &p : reconstruction_cloud) {
        auto originating_view = p.originating_views.begin();
        const int view_idx = originating_view->first;
        cv::Point2f p2d = features[view_idx].points[originating_view->second];
        cv::Vec3b color = images[view_idx].at<cv::Vec3b>(p2d);
        ofs << p.point.x << " " << p.point.y << " " << p.point.z << " "
            << (int) color(2) << " "
            << (int) color(1) << " "
            << (int) color(0) << " " << std::endl;
    }
    
    ofs.close();
}

void SfM::add_more_views_to_reconstruction() { 
    SFM_LOG("Adding more views to reconstruction...");
    
    while (done_views.size() != images.size()) {
        Image2D3DMatches matches_2d3d = find_2d_3d_matches();
        
        int best_view = -1;
        int best_num_matches = 0;
        for (const auto &match_2d3d : matches_2d3d) {
            int num_matches = (int) match_2d3d.second.points_2d.size();
            if (num_matches > best_num_matches) {
                best_view = match_2d3d.first;
                best_num_matches = num_matches;
            }
        }
        if (best_view == -1) {
            SFM_LOG("ERR! No 2D-3D matches has any legitimate match.");
            break;
        }
        SFM_LOG("Best view: %d has %d matches", best_view, best_num_matches);
        SFM_LOG("Trying to add %d to existing good views.", best_view);
        
        done_views.insert(best_view);
        
        // Retrieve new camera pose
        cv::Matx34f camera_pose;
        bool success = stereo_util.find_camera_pose_from_2d3d_match(intrinsics, matches_2d3d[best_view], camera_pose);
        if (!success) {
            SFM_LOG("Cannot retrieve camera pose for view: %d", best_view);
            continue;
        }

        camera_poses[best_view] = camera_pose;
        SFM_LOG("Camera pose for view %d located.", best_view);
        
        // Time to triangulate more points!
        bool any_view_success = false;
        for (int good_view : good_views) {
            int left = good_view < best_view ? good_view : best_view;
            int right = good_view < best_view ? best_view : good_view;
            Matches pruned_matches;
            
            cv::Matx34f p_left, p_right;
            bool success = stereo_util.find_camera_matrices_from_match(intrinsics, feature_match_mat[left][right], features[left], features[right], pruned_matches, p_left, p_right);
            feature_match_mat[left][right] = pruned_matches;
            if (!success) {
                SFM_LOG("EPIC FAIL: failde to get pruned matches");
                continue;
            }
            
            PointCloud cloud;
            success = stereo_util.triangulate_views(intrinsics, { left, right }, pruned_matches, features[left], features[right], camera_poses[left], camera_poses[right], cloud);
            if (!success) {
                SFM_LOG("Failed to trianglulate view %d and %d. Skipping...", left, right);
                continue;
            }
            
            SFM_LOG("Merging triangulation between %d and %d. Matches: #%lu", left, right, feature_match_mat[left][right].size());
            merge_new_pointclouds(cloud);
            any_view_success = true;
        }
        
        good_views.insert(best_view);
    }
}

SfM::Image2D3DMatches SfM::find_2d_3d_matches() { 
    Image2D3DMatches matches_2d3d;
    
    for (int view_idx = 0; view_idx < images.size(); view_idx++) {
        // If the view is done...
        if (done_views.find(view_idx) != done_views.end()) {
            // Skip it
            continue;
        }
        // Otherwise, for each point in the PC...
        Image2D3DMatch match_2d3d;
        for (const Point3DInMap &point : reconstruction_cloud) {
            bool found_point = false;
            
            // Find its originating view and view keypoint index
            for (const std::pair<int, int> view_and_idx : point.originating_views) {
                const int originating_view_idx = view_and_idx.first;
                const int keypoint_idx = view_and_idx.second;
                
                // Just kinda sort them, so it could be used by the feature matching matrix later
                const int left = (originating_view_idx < view_idx ? originating_view_idx : view_idx);
                const int right = (originating_view_idx < view_idx ? view_idx : originating_view_idx);
                
                for (const cv::DMatch &match : feature_match_mat[left][right]) {
                    int matched_2d_point_in_new_view = -1;
                    
                    // Is originating view "left"?
                    if (originating_view_idx < view_idx) {
                        if (match.queryIdx == keypoint_idx) {
                            matched_2d_point_in_new_view = match.trainIdx;
                        }
                    } else {
                        if (match.trainIdx == keypoint_idx) {
                            matched_2d_point_in_new_view = match.queryIdx;
                        }
                    }
                    // Is there such a point in this view?
                    if (matched_2d_point_in_new_view >= 0) {
                        const Features &view_features = features[view_idx];
                        match_2d3d.points_2d.push_back(view_features.points[matched_2d_point_in_new_view]);
                        match_2d3d.points_3d.push_back(point.point);
                        found_point = true;
                        break;
                    }
                } // for (const cv::DMatch &match : feature_match_mat[left][right])
                if (found_point) {
                    break;
                }
            } // for (const std::pair<int, int> &view_and_idx : point.originating_views)
        } // for (const Point3DInMap &point : reconstruction_cloud)
        matches_2d3d[view_idx] = match_2d3d;
    } // for (int view_idx = 0; view_idx < images.size(); view_idx++)
    return matches_2d3d;
}

void SfM::merge_new_pointclouds(const PointCloud &cloud) {
    // The more points there are, the better! Yay!
    reconstruction_cloud.insert(reconstruction_cloud.end(), cloud.begin(), cloud.end());
}



