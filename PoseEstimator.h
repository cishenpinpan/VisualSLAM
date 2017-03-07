//
//  PoseEstimator.hpp
//  VisualSLAM
//
//  Created by Rong Yuan on 3/4/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#ifndef PoseEstimator_h
#define PoseEstimator_h

#include <stdio.h>
#include "lmmin.h"
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
#include "Utility.h"
#include "Canvas.h"
#include <cvsba/cvsba.h>
#include <set>

using namespace std;
using namespace cv;

struct DataStruct
{
    View *v1, *v2;
    vector<Point3d> points3D;
    Mat R, T;
    double error, iterations;
    DataStruct(View *_v1, View *_v2, Mat _R, Mat _T, vector<Point3d> _points3D)
    {
        v1 = _v1;
        v2 = _v2;
        R = _R;
        T = _T;
        points3D = _points3D;
        error = 0.0;
        iterations = 0;
    }
};

class PoseEstimator
{
public:
    PoseEstimator();
    Mat estimatePose(View *v1, View *v2);
    double estimateScale(View *v1, View *v2);
private:
    Mat estimatePoseMono(View *v1, View *v2);
    FeatureExtractor *featureExtractor;
    FeatureTracker *featureTracker;
};
#endif /* PoseEstimator_h */
