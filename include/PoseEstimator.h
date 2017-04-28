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
#include <time.h>
#include <stdlib.h>
#include <algorithm>
#include <list>
#include "lmmin.h"
#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/core/affine.hpp"
#include "opencv2/calib3d/calib3d_c.h"
#include "opencv2/plot.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
//#include "g2o/math_groups/se3quat.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "View.h"
#include "CameraParameters.h"
#include "FeatureTracker.h"
#include "FeatureExtractor.h"
#include "Utility.h"
#include "Canvas.h"
#include "Converter.h"
#include "ViewTracker.h"
#include "MotionModel.h"
#include "SystemParameters.h"
//#include <cvsba/cvsba.h>
#include <set>

using namespace std;
using namespace cv;
using namespace Eigen;

struct PnP
{
    Mat pose;
    vector<KeyPoint> keyPoints;
    vector<Point3d> landmarks;
    vector<bool> status;
    double inlierRatio;
    PnP(const Mat _pose, const vector<KeyPoint> _keyPoints, const vector<Point3d> _landmarks)
    {
        pose = _pose;
        keyPoints = _keyPoints;
        landmarks = _landmarks;
    }
};
struct Triplet
{
    View *v;
    Mat prevPose;
    vector<KeyPoint> keyPoints;
    vector<Point3d> landmarks;
    vector<bool> status;
    double inlierRatio;
    Triplet(View *_v, const Mat _prevPose, vector<KeyPoint> _keyPoints, vector<Point3d> _landmarks)
    {
        v = _v;
        prevPose = _prevPose;
        keyPoints = _keyPoints;
        landmarks = _landmarks;
        status = vector<bool>(keyPoints.size(), true);
        inlierRatio = 0.0;
    }
};
struct Quadruple
{
    View *v1, *v2, *v3, *v4;
    vector<vector<KeyPoint>> keyPoints;
    vector<Point3d> landmarks;
    double error, inlierRatio;
    vector<bool> status;
    Quadruple(View *_v1, View *_v2, View *_v3, View *_v4,
                vector<vector<KeyPoint>> _keyPoints, vector<Point3d> _landmarks)
    {
        v1 = _v1;
        v2 = _v2;
        v3 = _v3;
        v4 = _v4;
        keyPoints = _keyPoints;
        landmarks = _landmarks;
        error = 0.0;
        inlierRatio = 1.0;
        status = vector<bool>(keyPoints.size(), false);
    }
};
class PoseEstimator
{
public:
    PoseEstimator();
    Mat estimatePose(View *v1, View *v2, double lambda = 1.0);
    Mat solvePnP(View *v, map<long, Landmark> landmarkBook);
    double solveScalePnP(View *v, const Mat prevPose,  map<long, Landmark> landmarkBook, double initial = 1.0);
    double solveScalePnPRANSAC(View *v, const Mat prevPose,  map<long, Landmark> landmarkBook, double initial = 1.0);
    double refineScale(vector<View*> views, map<long, Landmark> &landmarkBook);
    double refineScaleRANSAC(vector<View*> views, map<long, Landmark> &landmarkBook);
    ViewTracker* constructTriplet(View *v1, View *v2, View *v3);
    void solveRatioInTriplets(vector<View*> keyViews, vector<View*> allViews);
    void refineScaleStereo(vector<View*> views);
    void solvePosesPnPStereo(vector<View*> views);
    void refineScaleMultipleFramesWithDistribution(vector<View*> views, int N);
private:
    Mat estimatePoseMono(View *v1, View *v2, double lambda = 1.0);
    double refineScaleForThreeFrames(vector<View*> views, int i);
    void rejectOutliers(View *v1, View *v2, Mat status);
};
#endif /* PoseEstimator_h */
