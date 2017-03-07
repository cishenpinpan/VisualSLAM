//
//  CameraParameters.cpp
//  VisualSLAM
//
//  Created by Rong Yuan on 2/23/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#include "CameraParameters.h"

double CameraParameters::focal = 718.8650;
Point2d CameraParameters::principal = Point2d(607.1928, 185.2157);
Mat CameraParameters::getIntrinsic()
{
    Mat intrinsic = Mat::eye(3, 3, CV_64F);
    intrinsic.at<double>(0, 0) = CameraParameters::focal;
    intrinsic.at<double>(1, 1) = -CameraParameters::focal;
    intrinsic.at<double>(0, 2) = CameraParameters::principal.x;
    intrinsic.at<double>(1, 2) = 376 - CameraParameters::principal.y;
    return intrinsic.clone();
}

Mat CameraParameters::getStereoPose()
{
    Mat stereoPose = Mat::eye(4, 4, CV_64F);
    stereoPose.at<double>(0, 3) = 0.537;
    return stereoPose.clone();
}
