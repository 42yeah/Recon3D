//
//  Hoppe.cpp
//  hoppe
//
//  Created by apple on 21/03/2021.
//

#include <iostream>
#include <tiny_obj_loader.h>
#include <fstream>
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
        Vec3<float> point;
        point.x = attrib.vertices[i];
        point.y = attrib.vertices[i + 1];
        point.z = attrib.vertices[i + 2];
        pointcloud.push_back(point);
    }
    HOPPE_LOG("Pointcloud loading done. #vertices: %lu", pointcloud.size());
}

void Hoppe::run() {
    generate_tangent_plane();
}

void Hoppe::generate_tangent_plane() { 
    HOPPE_LOG("Generating tangent plane for each point in pointcloud...");
    
    for (int i = 0; i < pointcloud.size(); i++) {
        Neighborhood hood = { 0 };
        int num_neighborhood = get_neighborhood_in_pointcloud(i, hood);
        Vec3<float> centroid = calculate_centroid(i, hood, num_neighborhood);
        
    }
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
            const float dist_to_p = algebra::dist(pointcloud[idx], pointcloud[i]);
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

Vec3<float> Hoppe::calculate_centroid(int idx, Neighborhood hood, int n) { 
    Vec3<float> sum = { 0.0f, 0.0f, 0.0f };
    
    for (int i = 0; i < n; i++) {
        sum = sum + pointcloud[hood[n]];
    }
    sum = sum + pointcloud[idx];
    sum = sum / Vec3<float> { (float) n + 1,
                              (float) n + 1,
                              (float) n + 1 };
    return sum;
}

