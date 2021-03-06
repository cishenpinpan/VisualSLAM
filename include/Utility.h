//
//  Utility.hpp
//  VisualSLAM
//
//  Created by Rong Yuan on 3/4/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#ifndef Utility_h
#define Utility_h

#include "View.h"


void triangulatePoint(const Mat pose1, const Mat pose2, const KeyPoint point1, const KeyPoint point2, Point3d& point3d);

void triangulatePoints(Mat globalPose, Mat relativePose, vector<Point2d> points1,
                       vector<Point2d> points2, vector<Point3d> &points3D);

void triangulatePoints(Mat globalPose, Mat relativePose, vector<Point2f> points1,
                       vector<Point2f> points2, vector<Point3d> &points3D);

vector<double> rot2quat(const Mat R);
vector<double> rot2angles(const Mat &R);

Mat anglesToRotationMatrix(const Mat &theta);

pair<double, double> reproject3DPoint(Point3d point3d, Mat pose, KeyPoint point2d, bool L2Norm);
double reproject3DPoints(vector<Point3d> points3D, Mat pose, vector<Point2f> points2D);
Mat getEssentialMatrix(const Mat R, const Mat t);
double projectEpipolarLine(const Mat E, Point2f pixel1, Point2f pixel2);

double mean(const vector<double> &arr);
double median(const vector<double> &arr);
double standardDeviation(const vector<double> &arr);
void rejectDistributionOutliers(vector<double> &arr);

double huber(double err, double delta, bool L1Norm);


#endif /* Utility_h */
