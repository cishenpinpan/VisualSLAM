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
#include "View.h"
#include "CameraParameters.h"
#include "FeatureTracker.h"
#include "FeatureExtractor.h"
#include "Canvas.h"
#include <cvsba/cvsba.h>
#include <set>

using namespace std;
using namespace cv;

void triangulatePoints(Mat globalPose, Mat relativePose, vector<Point2d> points1,
                       vector<Point2d> points2, vector<Point3d> &points3D);

void triangulatePoints(Mat globalPose, Mat relativePose, vector<Point2f> points1,
                       vector<Point2f> points2, vector<Point3d> &points3D);

pair<double, double> reproject3DPoint(Point3d point3d, Mat pose, Point2f point2d, bool L2Norm);
double reproject3DPoints(vector<Point3d> points3D, Mat pose, vector<Point2f> points2D);
Mat getEssentialMatrix(const Mat R, const Mat t);
double projectEpipolarLine(const Mat E, Point2f pixel1, Point2f pixel2);

// singleton
class IdGenerator
{
private:
    long int nextId;
    static IdGenerator* instance;
    IdGenerator(){nextId = 0;};
public:
    static IdGenerator* createInstance()
    {
        if(!instance)
            instance = new IdGenerator();
        return instance;
    }
    long int next(){return nextId++;}
    vector<long int> next(const int length)
    {
        int len = length;
        vector<long int> res;
        while(len--)
            res.push_back(next());
        return res;
    }
};
           

#endif /* Utility_h */
