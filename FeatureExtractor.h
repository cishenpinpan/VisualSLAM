//
//  FeatureDetector.hpp
//  VisualSLAM
//
//  Created by Rong Yuan on 2/28/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#ifndef FeatureExtractor_h
#define FeatureExtractor_h

#include "opencv2/features2d.hpp"
#include "Feature.h"
#include "Factory.h"
#include <stdio.h>
#include <set>
#include <map>

using namespace std;
using namespace cv;

class FeatureExtractor {
public:
    vector<KeyPoint> extractFeatures(Mat img_1, FeatureSet& featureSet);
    vector<KeyPoint> reextractFeatures(Mat img_1, FeatureSet& featureSet);
};
#endif /* FeatureExtractor_h */
