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
#include "SystemParameters.h"

using namespace std;
using namespace cv;

class CameraParameters
{
public:
    static vector<double> focals;
    static vector<Point2d> principals;
    static vector<double> baselines;
    static vector<float> distCoeff;
    static double getFocal();
    static Point2d getPrincipal();
    static double getBaseline();
    static Mat getIntrinsic();
    static Mat getStereoPose();
    static Mat getDistCoeff();
};
#endif /* CameraParameters_h */
