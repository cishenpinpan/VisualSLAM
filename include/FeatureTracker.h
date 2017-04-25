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
#include "opencv2/imgcodecs.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "Canvas.h"
#include "FeatureCandidateExtractor.h"
#include "FeatureExtractor.h"
#include "Converter.h"

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

class FeatureTracker
{
public:
    void kltTrack(const Mat img1, const Mat img2, FeatureSet& featureSet1, FeatureSet& featureSet2, bool stereo);
    void refineTrackedFeatures(Mat img1, Mat img2, FeatureSet& featureSet1, FeatureSet& featureSet2, bool stereo);
    void trackAndMatch(vector<View*> views);
    void trackAndMatch(View *v);
    void track(vector<View*> views);
    void track(View *v);
};

#endif /* FeatureTracker_h */
