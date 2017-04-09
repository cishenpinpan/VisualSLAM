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
#include "SystemParameters.h"
#include <cvsba/cvsba.h>
#include <set>

using namespace std;
using namespace cv;
using namespace Eigen;

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
    Mat estimatePoseMotionOnlyBA(View *v1, View *v2, map<long, Landmark> landmarkBook);
    Mat solvePnP(View *v, map<long, Landmark> &landmarkBook);
private:
    Mat estimatePoseMono(View *v1, View *v2);
    FeatureExtractor *featureExtractor;
    FeatureTracker *featureTracker;
    void rejectOutliers(View *v1, View *v2, Mat status);
};
#endif /* PoseEstimator_h */
