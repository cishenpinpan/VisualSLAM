//
//  CameraParameters.hpp
//  VisualSLAM
//
//  Created by Rong Yuan on 2/23/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#ifndef CameraParameters_h
#define CameraParameters_h

#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class CameraParameters
{
public:
    static double focal;
    static Point2d principal;
    static vector<float> distCoeff;
    static Mat getIntrinsic();
    static Mat getStereoPose();
    static Mat getDistCoeff();
};
#endif /* CameraParameters_h */
