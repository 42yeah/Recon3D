//
//  algebra.hpp
//  hoppe
//
//  Created by apple on 21/03/2021.
//

#ifndef algebra_hpp
#define algebra_hpp

#define IN
#define OUT

#define K_NEIGHBORHOOD 8

#include <iostream>
#include <opencv2/core.hpp>


typedef int Neighborhood[K_NEIGHBORHOOD];


struct Plane {
    cv::Vec3f origin;
    cv::Vec3f normal;
};

#endif /* algebra_hpp */
