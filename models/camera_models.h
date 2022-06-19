#pragma once

#include <iostream>
#include "Eigen.h"

using namespace Eigen;

template <class T>
struct CameraParams{
    T fx;
    T fy;
    T cx;
    T cy;
};


template <class T> class CameraModel {
public:
    CameraModel() = default;
    virtual ~ CameraModel() = default;

    virtual void initialize(const CameraParams<T>& params) = 0;
    virtual Matrix<T,2,1> project (const Matrix<T,3,1>& vertex) const = 0;
    virtual Matrix<T,3,1> unproject (const Matrix<T,2,1>& pixel) const = 0;

};


template<class T>  class PinholeCamera: public CameraModel<T> {
private:
    T _fx, _fy, _cx, _cy;

public:
    PinholeCamera(){
        std::cout << "PinholeCamera constructor called" << std::endl;
    }
    
    ~PinholeCamera(){
        std::cout << "PinholeCamera destructor called" << std::endl;
    }
    
    void initialize(const CameraParams<T>& params) {

        _fx = params.fx;
        _fy = params.fy;
        _cx = params.cx;
        _cy = params.cy;

        std::cout <<"Pinhole camera model initialized!" << std::endl;
        std::cout << "_fx = " << _fx << std::endl;
        std::cout << "_fy = " << _fy << std::endl;
        std::cout << "_cx = " << _cx << std::endl;
        std::cout << "_cy = " << _cy << std::endl;
    }
    
    Matrix<T,2,1> project(const Matrix<T,3,1>& point) const {

        Matrix<T,2,1> pixel = Matrix<T,2,1>::Zero();
        
        pixel[0] = (_fx * point[0]) / point[2] + _cx;
        pixel[1] = (_fx * point[1]) / point[2] + _cy;
        
        return pixel;
    }

    Matrix<T,3,1> unproject(const Matrix<T,2,1>& pixel) const {

        Matrix<T, 3, 1> point3d = Matrix<T,3,1>::Zero();
        
        const T& mx = (pixel[0] - _cx) / _fx;
        const T& my = (pixel[1] - _cy) / _fy;
        const T& m  = std::sqrt(mx * mx + my * my + 1);

        point3d = (1 / m) *  Matrix<T,3,1>(mx,my,1);
        std::cout << "point3d = [" << point3d[0] << ", " << point3d[1] << ", " 
                  << point3d[2] << "]" << std::endl;

        return point3d;
    }   
    
};





// template<class T>
// class ExtendedUnifiedCamera: public CameraModel<T> {
    
// };

// template<class T>
// class DoubleSphereCamera: public CameraModel<T>{

// };

// template<class T>
// class KannalaBrandtCamera: public CameraModel<T>{

// };