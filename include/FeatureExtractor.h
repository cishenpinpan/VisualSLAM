//
//  FeatureDetector.hpp
//  VisualSLAM
//
//  Created by Rong Yuan on 2/28/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#ifndef FeatureExtractor_h
#define FeatureExtractor_h


#include "Feature.h"
#include "Factory.h"

using namespace cv::xfeatures2d;

namespace blindfind
{
    class FeatureExtractor {
    public:
        FeatureExtractor(){_featureName = FEATURE_NAME;};
        vector<KeyPoint> extractFeatures(Mat img_1, FeatureSet& featureSet);
        vector<KeyPoint> reextractFeatures(Mat img_1, FeatureSet& featureSet);
    private:
        string _featureName;
        
    };
}

#endif /* FeatureExtractor_h */
