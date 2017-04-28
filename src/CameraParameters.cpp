//
//  CameraParameters.cpp
//  VisualSLAM
//
//  Created by Rong Yuan on 2/23/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#include "CameraParameters.h"

vector<double> CameraParameters::focals
        = {718.856, 718.856, 718.856, 707.0912, 707.0912, 707.0912, 707.0912, 707.0912, 707.0912, 707.0912};
vector<Point2d> CameraParameters::principals
            = {Point2d(607.1928, 185.2157), Point2d(607.1928, 185.2157), Point2d(607.1928, 185.2157),
            Point2d(601.8873, 183.1104), Point2d(601.8873, 183.1104), Point2d(601.8873, 183.1104),
            Point2d(601.8873, 183.1104), Point2d(601.8873, 183.1104), Point2d(601.8873, 183.1104),
            Point2d(601.8873, 183.1104)};
vector<double> CameraParameters::baselines = {386.1448, 386.1448, 386.1448,
                    379.8145, 379.8145, 379.8145, 379.8145, 379.8145, 379.8145, 379.8145, 379.8145};
vector<float> CameraParameters::distCoeff = vector<float>(5, 0.0);
double CameraParameters::getFocal()
{
    int index = atoi(TRACK);
    return focals[index];
}
Point2d CameraParameters::getPrincipal()
{
    int index = atoi(TRACK);
    return principals[index];
}
double CameraParameters::getBaseline()
{
    int index = atoi(TRACK);
    return baselines[index];
}
Mat CameraParameters::getIntrinsic()
{
    Mat intrinsic = Mat::eye(3, 3, CV_64F);
    intrinsic.at<double>(0, 0) = getFocal();
    intrinsic.at<double>(1, 1) = getFocal();
    intrinsic.at<double>(0, 2) = getPrincipal().x;
    intrinsic.at<double>(1, 2) = getPrincipal().y;
    return intrinsic.clone();
}

Mat CameraParameters::getStereoPose()
{
    Mat stereoPose = Mat::eye(4, 4, CV_64F);
    stereoPose.at<double>(0, 3) = CameraParameters::getBaseline() / getFocal();
    return stereoPose.clone();
}

Mat CameraParameters::getDistCoeff()
{
    Mat distCoeffMat(5, 1, CV_64F);
    for(int i = 0; i < 5; i++)
        distCoeffMat.at<double>(i, 0) = distCoeff[i];
    return distCoeffMat.clone();
}
