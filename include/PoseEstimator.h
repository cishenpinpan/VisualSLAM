//
//  PoseEstimator.hpp
//  VisualSLAM
//
//  Created by Rong Yuan on 3/4/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#ifndef PoseEstimator_h
#define PoseEstimator_h

#include "lmmin.h"
#include "View.h"
#include "FeatureTracker.h"
#include "Utility.h"
#include "ViewTracker.h"

namespace blindfind
{
    class View;
    class Landmark;
    class PoseEstimator
    {
    public:
        Mat estimatePose(View *v1, View *v2, bool right = false);
        Mat solvePnP(View *v, map<long, Landmark> landmarkBook);
        void estimateScaleQuadrupleRANSAC(View *prev, View *stereo, View *next, const Mat pose21,
                                          const Mat pose23, double *lambda1, double *lambda2, map<long, Landmark> landmarkBook);
        double estimateScaleTrinocularRANSAC(View *stereo, View *v);
        double solveScalePnP(View *v, const Mat prevPose,  map<long, Landmark> landmarkBook, double initial = 1.0);
        double solveScalePnPRANSAC(View *v, const Mat prevPose,  map<long, Landmark> landmarkBook, double initial = 1.0);
        ViewTracker* constructTriplet(View *v1, View *v2, View *v3);
        void solveRatioInTriplets(vector<View*> keyViews, vector<View*> allViews);
        void solveScale(vector<View*> allViews, vector<View*> keyViews, bool trinocular);
        void solvePosesPnPStereo(vector<View*> views);
        void refineScaleMultipleFramesWithDistribution(vector<View*> views, int N);
    private:
        Mat estimatePoseMono(View *v1, View *v2, bool right = false);
        double refineScaleForThreeFrames(vector<View*> views, int i);
        void rejectOutliers(View *v1, View *v2, Mat status);
    };
}

#endif /* PoseEstimator_h */
