//
// Created by nagesh
//

#include <iostream>

using namespace std;


#ifndef RGBD_SALM_PARAMS_H
#define RGBD_SALM_PARAMS_H


struct CAMERA {
    double fx = 525.0;  // focal length x
    double fy = 525.0;  // focal length y
    double cx = 319.5; //optical center x
    double cy = 239.5 ; // optical center y
    double factor = 5000 ;// for the 16-bit PNG files
}camera;

#endif //RGBD_SALM_PARAMS_H
