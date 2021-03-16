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
        LOG("There is no image to work on.");
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
    
    return OK;
}

bool SfM::set_image_dir(std::string path) {
    LOG("Setting image directory as %s", path.c_str());
    try {
        for (const auto &entry : std::filesystem::directory_iterator(path)) {
            cv::Mat img = cv::imread(entry.path());
            if (img.rows == 0 || img.cols == 0) {
                LOG("WARNING! Invalid image: %s. Skipping...", entry.path().c_str());
                continue;
            }
            images.push_back(std::move(img));
        }
    } catch (std::exception &e) {
        LOG("Failed to load image directory.");
        return false;
    }
    return true;
}

void SfM::extract_features() { 
    LOG("Extracting features...");
    
    features.resize(images.size());
    for (int i = 0; i < images.size(); i++) {
        features[i] = feature_util.extract_features(images[i]);
        LOG("Image %d has %ld keypoints", i, features[i].points.size());
    }
}

void SfM::create_feature_match_matrix() {
    LOG("Creating feature match matrix...");
    
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
    LOG("Launching %d threads with %d pairs per thread...", num_threads, num_pairs_for_thread);
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
                LOG("Thread %d -> pair %d: %ld matches", thread_id, pair_id, feature_match_mat[pair.left][pair.right].size());
                write_mutex.unlock();
            }
        }));
    }
    
    for (auto &t : threads) {
        t.join();
    }
}

void SfM::find_baseline_triangulation() { 
    LOG("Finding baseline triangulation...");

    std::map<float, ImagePair> homography_inliers = sort_views_for_baseline();
    
    PointCloud cloud;
    
    LOG("Trying views in triangulation...");
    for (auto &pair : homography_inliers) {
        LOG("Trying (%d, %d) ratio: %f", pair.second.left, pair.second.right, pair.first);
        
        cv::Matx34f p_left = cv::Matx34f::eye();
        cv::Matx34f p_right = cv::Matx34f::eye();
        int left = pair.second.left;
        int right = pair.second.right;

        Matches pruned_matches;
        
        LOG("Finding camera matrices...");
        bool success = stereo_util.find_camera_matrices_from_match(intrinsics, feature_match_mat[left][right], features[left], features[right], pruned_matches, p_left, p_right);
        if (!success) {
            LOG("WARNING! Camera matrix recovery failed.");
        }
        
        float pose_inlier_ratio = (float) pruned_matches.size() / feature_match_mat[left][right].size();
        LOG("Pruned inlier ratio of (%d, %d): %f", left, right, pose_inlier_ratio);
        if (pose_inlier_ratio < POSE_INLIER_MINIMAL_RATIO) {
            LOG("Insufficient pose inliers. Skipping.");
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
        
        LOG("Triangulating from stereo views (%d, %d)...", left, right);
        success = stereo_util.triangulate_views(intrinsics, pair.second, feature_match_mat[left][right], features[left], features[right], p_left, p_right, cloud);
        if (!success) {
            LOG("Could not triangulate: (%d, %d).", left, right);
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
    LOG("Finding views homography inliers...");
    
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
            LOG("Homography inlier ratio: %d %d %f", i, j, inliers_ratio);
        }
    }
    return matches_sizes;
}

void SfM::export_to_ply(std::string path) { 
    LOG("Saving point cloud to %s", path.c_str());
    
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



