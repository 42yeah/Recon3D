//
//  Hoppe.hpp
//  hoppe
//  This is an implementation of Hoppe's method on
//  Surface Reconstruction from Unorganized Points.
//
//  Created by apple on 21/03/2021.
//

#ifndef hoppe_
#define hoppe_

/* The classes below are exported */
#pragma GCC visibility push(default)

#define HOPPE_LOG_LEVEL 1

#if HOPPE_LOG_LEVEL == 0
#define HOPPE_LOG(...)
#elif HOPPE_LOG_LEVEL == 1
#define HOPPE_LOG(fmt, ...) printf(fmt "\n", ##__VA_ARGS__)
#elif HOPPE_LOG_LEVEL == 2
#define HOPPE_LOG(fmt, ...) printf("%s:%d: " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__)
#endif

#include <vector>
#include "algebra.hpp"


class Hoppe {
public:
    Hoppe() {}
    
    ~Hoppe() = default;
    
    void load_pointcloud(std::string path);
    
    void run();

private:
    void generate_tangent_plane();
    
    /// Get K-neighborhood in pointcloud. K is defined in algebra.hpp.
    /// @param idx the index to fetch neighborhood from
    /// @param hood the return array
    int get_neighborhood_in_pointcloud(IN int idx,
                                       OUT Neighborhood hood);
    
    
    /// Calculates centroid of neighborhood as origin of tangent plane.
    /// Maybe averaging them is enough?
    /// @param hood the point's K-neighborhood
    /// @param idx the index of p (the host)
    /// @param n number of neighborhood
    Vec3<float> calculate_centroid(int idx, Neighborhood hood, int n);

    std::vector<Vec3<float> > pointcloud;
    std::vector<Plane> tangent_plane;
    
    // TODO: Add variables hoppe needs
};

#pragma GCC visibility pop
#endif
