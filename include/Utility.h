//
//  Utility.hpp
//  VisualSLAM
//
//  Created by Rong Yuan on 3/4/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#ifndef Utility_h
#define Utility_h

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/core/affine.hpp"
#include "opencv2/calib3d/calib3d_c.h"
#include "opencv2/plot.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "CameraParameters.h"
#include "FeatureTracker.h"
#include "FeatureExtractor.h"
#include "Canvas.h"
//#include <cvsba/cvsba.h>
#include <set>

using namespace std;
using namespace cv;

void triangulatePoint(const Mat pose1, const Mat pose2, const KeyPoint point1, const KeyPoint point2, Point3d& point3d);

void triangulatePoints(Mat globalPose, Mat relativePose, vector<Point2d> points1,
                       vector<Point2d> points2, vector<Point3d> &points3D);

void triangulatePoints(Mat globalPose, Mat relativePose, vector<Point2f> points1,
                       vector<Point2f> points2, vector<Point3d> &points3D);

vector<double> rot2quat(const Mat R);

Mat anglesToRotationMatrix(const Mat &theta);

pair<double, double> reproject3DPoint(Point3d point3d, Mat pose, KeyPoint point2d, bool L2Norm);
double reproject3DPoints(vector<Point3d> points3D, Mat pose, vector<Point2f> points2D);
Mat getEssentialMatrix(const Mat R, const Mat t);
double projectEpipolarLine(const Mat E, Point2f pixel1, Point2f pixel2);


           

#endif /* Utility_h */
