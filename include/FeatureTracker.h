//
//  FeatureTracker.hpp
//  VisualSLAM
//
//  Created by Rong Yuan on 2/28/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#ifndef FeatureTracker_h
#define FeatureTracker_h

#include <time.h>
#include "View.h"
#include "Canvas.h"
#include "FeatureCandidateExtractor.h"
#include "FeatureExtractor.h"

namespace blindfind
{
    class View;
    class FeatureTracker
    {
    public:
        // when tracking or matching, only the last view is modified
        void kltTrack(const Mat img1, const Mat img2, const FeatureSet& featureSet1,FeatureSet& featureSet2);
        void refineTrackedFeatures(Mat img1, Mat img2, const FeatureSet& featureSet1, FeatureSet& featureSet2);
        // trackAndMatch locks tracked feature in real corners
        // when passing in a single view, it is matching within a stereo
        void trackAndMatch(vector<View*> views);
        void trackAndMatch(View *v);
        void track(vector<View*> views);
        void track(View *v);
    };
}


#endif /* FeatureTracker_h */
