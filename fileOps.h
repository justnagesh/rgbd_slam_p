//
// Created by nagesh
//

#include "iostream"
#include <vector>
#include <fstream>
#include <sstream>

using namespace std;

#ifndef RGBD_SALM_FILEOPS_H
#define RGBD_SALM_FILEOPS_H


class fileOps {
public:
    vector < string > depth_names;
    vector < string > rgb_names;
    vector < string > pcd_file_names;
    fileOps();
};


#endif //RGBD_SALM_FILEOPS_H
