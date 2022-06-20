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
    
    T alpha;
    T beta;
    
    T xi;

    T k1;
    T k2;
    T k3;
    T k4;
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
    PinholeCamera(){};
   ~PinholeCamera(){};
    
    void initialize(const CameraParams<T>& params) {

        _fx = params.fx;
        _fy = params.fy;
        _cx = params.cx;
        _cy = params.cy;

        std::cout <<"Pinhole camera model initialized!" << std::endl;
        std::cout << "fx = " << _fx << std::endl;
        std::cout << "fy = " << _fy << std::endl;
        std::cout << "cx = " << _cx << std::endl;
        std::cout << "cy = " << _cy << std::endl;
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
        const T&  m = std::sqrt(mx * mx + my * my + 1);

        point3d = (1 / m) * Matrix<T,3,1>(mx, my, 1);
        return point3d;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


template<class T> class ExtendedUnifiedCamera: public CameraModel<T> {
private:
    T _fx, _fy, _cx, _cy, _alpha, _beta;
public:  
    ExtendedUnifiedCamera();
   ~ExtendedUnifiedCamera();

    void initialize(const CameraParams<T>& params) {
        _fx = params.fx;
        _fy = params.fy;
        _cx = params.cx;
        _cy = params.cy;
        _alpha = params.alpha;
        _beta  = params.beta;

        std::cout << "Extended Unified Camera model initialized" << std::endl;
        std::cout << "fx = " << _fx << std::endl;
        std::cout << "fy = " << _fy << std::endl;
        std::cout << "cx = " << _cx << std::endl;
        std::cout << "cy = " << _cy << std::endl;
        std::cout << "alpha = " << _alpha << std::endl;
        std::cout << "beta  = " << _beta  << std::endl;
    }

    Matrix<T,2,1> project(const Matrix<T,3,1>& point) const {
        
        Matrix<T,2,1> pixel = Matrix<T,2,1>::Zero();

        const T& x = point[0];
        const T& y = point[1];
        const T& z = point[2];

        const T& d = std::sqrt(_beta * (x*x + y*y) + z*z);

        pixel[0] = (_fx * x) / (_alpha * d + (1 - _alpha) * z) + _cx;
        pixel[1] = (_fy * y) / (_alpha * d + (1 - _alpha) * z) + _cy;

        return pixel;
    }

    Matrix<T,3,1> unproject(const Matrix<T,2,1>& pixel) const {

        Matrix<T,3,1> point3d = Matrix<T,3,1>::Zero();
        
        const T& mx = (pixel[0] - _cx) / _fx;
        const T& my = (pixel[1] - _cy) / _fy;

        const T& r_squared = mx * mx + my * my;
        const T& mz = (1 - _beta * _alpha * _alpha * r_squared) /
                      (_alpha * std::sqrt(1 - (2 * _alpha -1) * _beta * r_squared) +
                      (1 - _alpha));

        point3d = 1 / std::sqrt(mx * mx + my * my + mz * mz) * Matrix<T,3,1>(mx, my, mz);

        return point3d;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


template<class T> class DoubleSphereCamera: public CameraModel<T>{
private:
    T _fx, _fy, _cx, _cy, _xi, _alpha;

public:
    DoubleSphereCamera();
   ~DoubleSphereCamera();

    void initialize(const CameraParams<T>& params) {

        _fx = params.fx;
        _fy = params.fy;
        _cx = params.cx;
        _cy = params.cy;
        _xi = params.xi;
        _alpha = params.alpha;

        std::cout << "Double sphere camera model initialized!" << std::endl;
        std::cout << "fx = " << _fx << std::endl;
        std::cout << "fy = " << _fy << std::endl;
        std::cout << "cx = " << _cx << std::endl;
        std::cout << "cy = " << _cy << std::endl;
        std::cout << "xi = " << _xi << std::endl;
        std::cout << "alpha = " << _alpha << std::endl;
    }

    Matrix<T,2,1> project(const Matrix<T,3,1>& point) const {

        Matrix<T,2,1> pixel = Matrix<T,2,1>::Zero();

        const T& x = point[0];
        const T& y = point[1];
        const T& z = point[2];

        const T& d1 = std::sqrt(x * x + y * y + z * z);
        const T& d2 = std::sqrt(x * x + y * y + (_xi * d1 + z) * (_xi * d1 + z));

        pixel[0] = (_fx * x) / (_alpha * d2 + (1 - _alpha) * (_xi * d1 + z)) + _cx;
        pixel[1] = (_fy * y) / (_alpha * d2 + (1 - _alpha) * (_xi * d1 + z)) + _cy;

        return pixel;
    }

    Matrix<T,3,1> unproject(const Matrix<T,2,1>& pixel) const {

        Matrix<T,3,1> point3d = Matrix<T,3,1>::Zero();

        const T& mx = (pixel[0] - _cx) / _fx;
        const T& my = (pixel[1] - _cy) / _fy;

        const T& r_squared = mx * mx + my * my;
        const T& mz = (1 - _alpha * _alpha * r_squared) /
                      (_alpha * std::sqrt(1 - (2 * _alpha - 1) * r_squared) + 1 - _alpha);

        point3d = (mz * _xi + std::sqrt(mz * mz + (1 - _xi * _xi) * r_squared)) /
                  (mz * mz + r_squared) * Matrix<T,3,1>(mx, my, mz) - Matrix<T,3,1>(0, 0, _xi);

        return point3d;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<class T> class KannalaBrandtCamera: public CameraModel<T>{
private:
    T _fx, _fy, _cx, _cy, _k1, _k2, _k3, _k4;

public:
    KannalaBrandtCamera();
   ~KannalaBrandtCamera();

    void initialize(const CameraParams<T>& params) {
        _fx = params.fx;
        _fy = params.fy;
        _cx = params.cx;
        _cy = params.cy;
        _k1 = params.k1;
        _k2 = params.k2;
        _k3 = params.k3;
        _k4 = params.k4;

        std::cout << "Kannala Brandt camera model initialized!" << std::endl;
        std::cout << "fx = " << _fx << std::endl;
        std::cout << "fy = " << _fy << std::endl;
        std::cout << "cx = " << _cx << std::endl;
        std::cout << "cy = " << _cy << std::endl;
        std::cout << "k1 = " << _k1 << std::endl;
        std::cout << "k2 = " << _k2 << std::endl;
        std::cout << "k3 = " << _k3 << std::endl;
        std::cout << "k4 = " << _k4 << std::endl;

    }

    Matrix<T,2,1> project(const Matrix<T,3,1>& point) const {
        
        Matrix<T,2,1> pixel = Matrix<T,2,1>::Zero();

        const T& x = point[0];
        const T& y = point[1];
        const T& z = point[2];

        const T& eps = 1e-20;

        const T& r = std::sqrt(x * x + y * y) + eps;
        const T& theta = std::atan2(r, z);

        T poly_val = 0;
        Matrix<T,10,1> k;

        k << _k4, 0, _k3, 0, _k2, 0, _k1, 0, 1, 0;

        // calculate polynomial with Horner's method 
        for (auto i = 0; i < 10; i++) {
            poly_val = (poly_val * theta) + k[i];
        }

        pixel[0] = _fx * poly_val * x / r + _cx;
        pixel[1] = _fy * poly_val * y / r + _cy;

        return pixel;
    }
    
    Matrix<T,3,1> unproject(const Matrix<T,2,1>& pixel) const {

        Matrix<T,3,1> point3d = Matrix<T,3,1>::Zero();

        const T& mx = (pixel[0] - _cx) / _fx;
        const T& my = (pixel[1] - _cy) / _fy;

        const T& eps = 1e-13;

        const T& r = std::sqrt(mx * mx + my * my) + eps;

        Matrix<T,10,1> k;
        Matrix<T,9,1> k_prime;

        k << _k4, 0, _k3, 0, _k2, 0, _k1, 0, 1, -r;
        k_prime << 9 * _k4, 0, 7 * _k3, 0, 5 * _k2, 0, 3 * _k1, 0, 1;

        //Newton's method to find polynomial roots
        const T& iters = 10;
        T theta_prev = 0.75;  // set initial value
        bool converged = false;

        T theta, poly_eval, poly_prime_eval;

        for (auto i = 0; i < iters; i++) {
            poly_eval = 0;
            poly_prime_eval = 0;

            for (auto i = 0; i < 10; i++) {
                poly_eval = (poly_eval * theta_prev) + k[i];
            }  

            for (auto i = 0; i < 9; i++) {
                poly_prime_eval = (poly_prime_eval * theta_prev) + k_prime[i];
            }

            theta = theta_prev - poly_eval / poly_prime_eval;

            poly_eval = 0; 
            for (auto i = 0; i < 10; i++) {
                poly_eval = (poly_eval * theta) + k[i];
            }

            if (std::fabs(poly_eval) < 1e-7) {
                converged = true;
                break;
            } else {
                theta_prev = theta;
            }
        }

        if (converged) {
            point3d = Matrix<T,3,1> ( std::sin(theta) * mx / r , std::sin(theta) * my / r,
                                      std::cos(theta) );
        } else {
            std::cout << "Newton's method did not converge!! Try either more iters or change initial value" << std::endl;
        }
        return point3d;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};