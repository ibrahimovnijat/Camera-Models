#include <iostream>
#include "models/camera_models.h" 
#include <memory>

using namespace std;

int main(){
    
    // declate camera parameters (also alpha/beta and k vals for other models)
    CameraParams<double> params;
    params.fx = 525;
    params.fy = 525;
    params.cx = 319.5;
    params.cy = 239.5;

    //e.g. 3D point in the camera coordinate
    Matrix<double,3,1> point3d (3,5,1);

    Matrix<double,2,1> pixel;
    Matrix<double,3,1> p_reproj;

    // Crate an instance of camera with Pinhole model
    unique_ptr<CameraModel<double>> camModel = make_unique<PinholeCamera<double>> ();
    camModel->initialize(params);

    pixel = camModel->project(point3d);
    p_reproj = camModel->unproject(pixel);

    cout << "Point: [" << point3d[0] << ", " << point3d[1] << ", " << point3d[2] << "]" << endl;
    cout << "Pixel: [" << pixel[0] << ", " << pixel[1] << "]" << endl;
    cout << "Reprojected point (normalized): [" <<p_reproj[0] << ", " << p_reproj[1] << ", " << p_reproj[2] << "]" << endl;

    return 0;
}