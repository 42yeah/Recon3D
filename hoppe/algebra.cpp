//
//  algebra.cpp
//  hoppe
//
//  Created by apple on 21/03/2021.
//

#include "algebra.hpp"


namespace algebra {

float dot(const Vec3<float> &thiz, const Vec3<float> &that) {
    return thiz.x * that.x + thiz.y * that.y + thiz.z * that.z;
}

Matrix<float> outer(const Vec3<float> &thiz, const Vec3<float> &that) {
    Matrix<float> ret(3, 3);
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ret.set(i, j, thiz[i] * that[j]);
        }
    }
    return ret;
}

float dist(const Vec3<float> &thiz, const Vec3<float> &that) {
    const Vec3<float> dir = thiz - that;
    return sqrtf(dot(dir, dir));
}

};
