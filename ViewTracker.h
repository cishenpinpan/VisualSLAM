//
//  FeatureTracker.h
//  VisualSLAM
//
//  Created by Rong Yuan on 2/24/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#ifndef ViewTracker_h
#define ViewTracker_h
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

using namespace cvsba;

class ViewTracker
{
public:
    ViewTracker();
    ~ViewTracker();
    void addView(View *v);
    vector<View*> getViews();
    void bundleAdjust();
    View* getPrevView();
    View* getCurrView();
    View* popLastView();
    void rejectOutliers(const Mat _mask);
    
private:
    
    FeatureTracker *featureTracker;
    FeatureExtractor *featureExtractor;
    vector<View*> views;
    void trackFeatures(View *v1, View *v2);
    set<int> findSurvivingFids();
    double reproject3DPoints(vector<Point3d> points3D, Mat pose, vector<Point2f> points2D);
};

#endif /* ViewTracker_h */
