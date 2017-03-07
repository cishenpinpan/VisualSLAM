//
//  FeatureTracker.hpp
//  VisualSLAM
//
//  Created by Rong Yuan on 2/28/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#ifndef FeatureTracker_h
#define FeatureTracker_h

#include "View.h"
#include "Canvas.h"

using namespace std;

class FeatureTracker
{
public:
    // general case
    vector<uchar> track(Mat img1, Mat img2, FeatureSet& featureSet1,
                        FeatureSet& featureSet2, bool stereo);
    // stereo
    void track(View *v);
    // temporal
    void track(View *v1, View *v2, bool camera);
private:
    void updateFeatures(vector<Point2f> &points, map<int, int> currFids, map<int, int> targetFids);
};

#endif /* FeatureTracker_h */
