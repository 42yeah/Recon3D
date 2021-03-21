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


typedef int Neighborhood[K_NEIGHBORHOOD];

template<typename T>
struct Vec3 {
    T operator[](int idx) const {
        switch (idx) {
            case 0:
                return x;
                
            case 1:
                return y;
                
            case 2:
                return z;
                
            default:
                throw "Index out of bound: " + std::to_string(idx);
        }
    }
    
    T x, y, z;
};


template<typename T>
Vec3<T> operator+(const Vec3<T> &thiz, const Vec3<T> &that) {
    Vec3<T> new_vec3 = {
        thiz.x + that.x,
        thiz.y + that.y,
        thiz.z + that.z
    };
    return new_vec3;
}

template<typename T>
Vec3<T> operator-(const Vec3<T> &thiz, const Vec3<T> &that) {
    Vec3<T> new_vec3 = {
        thiz.x - that.x,
        thiz.y - that.y,
        thiz.z - that.z
    };
    return new_vec3;
}

template<typename T>
Vec3<T> operator*(const Vec3<T> &thiz, const Vec3<T> &that) {
    Vec3<T> new_vec3 = {
        thiz.x * that.x,
        thiz.y * that.y,
        thiz.z * that.z
    };
    return new_vec3;
}

template<typename T>
Vec3<T> operator/(const Vec3<T> &thiz, const Vec3<T> &that) {
    Vec3<T> new_vec3 = {
        thiz.x / that.x,
        thiz.y / that.y,
        thiz.z / that.z
    };
    return new_vec3;
}


template<typename T>
struct Matrix {
    Matrix(int rows, int cols) : rows(rows), cols(cols) {
        data = new T[rows * cols];
    }

    Matrix(Matrix &&mat) : data(mat.data), rows(mat.rows), cols(mat.cols) {
        mat.data = nullptr;
    };

    ~Matrix() {
        if (data != nullptr) {
            delete[] data;
        }
    }
    
    T at(int row, int col) {
        if (row < 0 || col < 0 || row >= rows || col >= cols) {
            throw std::string("Trying to visit ") + std::to_string(row) + ", " + std::to_string(col);
        }
        return data[row * cols + col];
    }
    
    void set(int row, int col, T val) {
        if (row < 0 || col < 0 || row >= rows || col >= cols) {
            throw std::string("Trying to visit ") + std::to_string(row) + ", " + std::to_string(col);
        }
        data[row * cols + col] = val;
    }
    
    T *operator[](int row);

    T *data;
    int rows, cols;
};


struct Plane {
    Vec3<float> origin;
    Vec3<float> normal;
};


namespace algebra {

float dot(const Vec3<float> &thiz, const Vec3<float> &that);

Matrix<float> outer(const Vec3<float> &thiz, const Vec3<float> &that);

float dist(const Vec3<float> &thiz, const Vec3<float> &that);

};

#endif /* algebra_hpp */
