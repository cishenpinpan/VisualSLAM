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
    // temporal / general
    Mat track(Mat img1, Mat img2, FeatureSet& featureSet1,
                        FeatureSet& featureSet2, bool stereo);
    Mat searchLandmarks(View* v, map<long, Landmark> landmarkBook, View* prevView);

	vector<long int> kltTrack(Mat img1, Mat img2, FeatureSet& featureSet1, FeatureSet& featureSet2, bool stereo);

	void refineTrackedFeatures(Mat img1, Mat img2, FeatureSet& featureSet1, FeatureSet& featureSet2, vector<long int> idContainer, bool stereo);
    // stereo
    void track(View *v);

private:
    void updateFeatures(vector<Point2f> &points, map<int, int> currFids, map<int, int> targetFids);
    
};

#endif /* FeatureTracker_h */
