#include <iostream>
#include "models/camera_models.h" 
#include <memory>

using namespace std;


int main(){
    // unique_ptr<CameraModel<double>> camModel = make_unique<PinholeCamera<double>>();    
    shared_ptr<CameraModel<float>> camModel = make_shared<PinholeCamera<float>>();



    CameraParams<float> params;
    params.fx = 200;
    params.fy = 200;
    params.cx = 250;
    params.cy = 250;

    camModel->initialize(params);

    Matrix<float,3,1> vertex (5,4,2);
    Matrix<float,2,1> pixels;

    pixels = camModel->project(vertex);
    
    cout << "Pixel: \n" << pixels << endl;
    cout << "Unproject: \n" << camModel->unproject(pixels) << endl;

    return 0;
}