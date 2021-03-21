//
//  Hoppe.cpp
//  hoppe
//
//  Created by apple on 21/03/2021.
//

#include <iostream>
#include <tiny_obj_loader.h>
#include <fstream>
#include <thread>
#include "Hoppe.hpp"


void Hoppe::load_pointcloud(std::string path) {
    HOPPE_LOG("Loading pointcloud from %s...", path.c_str());
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    
    std::string warn, err;
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, path.c_str());
    
    if (!ret) {
        HOPPE_LOG("Failed to load pointcloud.");
        return;
    }
    
    pointcloud.clear();
    for (int i = 0; i < attrib.vertices.size(); i += 3) {
        cv::Vec3f point;
        point[0] = attrib.vertices[i];
        point[1] = attrib.vertices[i + 1];
        point[2] = attrib.vertices[i + 2];
        pointcloud.push_back(point);
    }
    HOPPE_LOG("Pointcloud loading done. #vertices: %lu", pointcloud.size());
}

void Hoppe::run() {
    generate_tangent_plane();
}

void Hoppe::generate_tangent_plane() { 
    HOPPE_LOG("Generating tangent plane for each point in pointcloud...");
    
    int max_threads = std::min((int) std::thread::hardware_concurrency(), (int) pointcloud.size());
    int pc_per_thread = (int) ceil(pointcloud.size() / (max_threads - 1));
    std::vector<std::thread> threads;
    std::mutex write_mutex;

    for (int thread_id = 0; thread_id < max_threads; thread_id++) {
        HOPPE_LOG("Generating thread from %d to %d", pc_per_thread * thread_id, std::min(pc_per_thread * (thread_id + 1), (int) pointcloud.size()));
        threads.push_back(std::thread([&, thread_id] () {

            for (int offset = 0; offset < pc_per_thread; offset++) {
                const int i = pc_per_thread * thread_id + offset;
                if (i >= pointcloud.size()) {
                    break;
                }
                Neighborhood hood = { 0 };
                int num_neighborhood = get_neighborhood_in_pointcloud(i, hood);
                cv::Vec3f centroid = calculate_centroid(i, hood, num_neighborhood);
                cv::Matx33f covariance_mat = cv::Matx33f::zeros();
                
                for (int neighbor = 0; neighbor < num_neighborhood; neighbor++) {
                    cv::Vec3f neighbor_p = pointcloud[hood[neighbor]];
                    cv::Vec3f dir = neighbor_p - centroid;
                    cv::Matx33f cv;
                    cv::mulTransposed(dir, cv, false);
                    covariance_mat += cv;
                }
                
                // Find eigenvector of this one
                cv::Vec3f eigenvalues;
                cv::Matx33f eigenvectors;
                
                write_mutex.lock();
                cv::eigen(covariance_mat, eigenvalues, eigenvectors);
                write_mutex.unlock();
                
                cv::Vec3f normal = {
                    eigenvectors(2, 0),
                    eigenvectors(2, 1),
                    eigenvectors(2, 2),
                };
                normal = cv::normalize(normal);
                
                Plane tangent_plane;
                tangent_plane.origin = centroid;
                tangent_plane.normal = normal;
                
                write_mutex.lock();
                tangent_planes.push_back(tangent_plane);
                write_mutex.unlock();
            }

        }));
    }
    
    for (std::thread &t : threads) {
        t.join();
    }
    
    HOPPE_LOG("Tangent plane calculation done: #%lu", tangent_planes.size());
}

int Hoppe::get_neighborhood_in_pointcloud(int idx, Neighborhood hood) { 
    int current_nbhd_idx = 0;
    
    if (pointcloud.size() <= K_NEIGHBORHOOD) {
        HOPPE_LOG("WARNING! The pointcloud is too small. It's pointless.");
    }
    
    while (current_nbhd_idx < K_NEIGHBORHOOD) {
        int closest_idx = -1;
        float closest_dist = 0.0f;
        for (int i = 0; i < pointcloud.size(); i++) {
            const float dist_to_p = cv::norm(pointcloud[idx] - pointcloud[i]);
            // If it's not the index itself, AND
            // closest_idx is not defined yet, OR its distance is closer to closest_dist
            if ((idx != i) && (closest_idx == -1 || dist_to_p < closest_dist)) {
                // AND it is not in hood already
                bool found = false;
                for (int j = 0; j < current_nbhd_idx; j++) {
                    if (hood[j] == i) {
                        found = true;
                        break;
                    }
                }
                if (found) {
                    continue;
                }
                // Mark it as closest_idx.
                closest_idx = i;
                closest_dist = dist_to_p;
            }
        }
        if (closest_idx != -1) {
            hood[current_nbhd_idx++] = closest_idx;
        }
    }
    return current_nbhd_idx;
}

cv::Vec3f Hoppe::calculate_centroid(int idx, Neighborhood hood, int n) {
    cv::Vec3f sum = { 0.0f, 0.0f, 0.0f };
    
    for (int i = 0; i < n; i++) {
        sum += pointcloud[hood[i]];
    }
    sum += pointcloud[idx];
    sum /= (float) (n + 1);
    return sum;
}

