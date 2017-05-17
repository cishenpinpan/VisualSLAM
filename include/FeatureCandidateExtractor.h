//
//  FeatureCandidateDetector.hpp
//  VisualSLAM
//
//  Created by Rong Yuan on 4/14/17.
//  Copyright © 2017 Hongyi Fan. All rights reserved.
//

#ifndef FeatureCandidateExtractor_h
#define FeatureCandidateExtractor_h

#include "Feature.h"
#include "Factory.h"

using namespace cv::xfeatures2d;

namespace blindfind
{
    class FeatureCandidateExtractor {
    public:
        FeatureCandidateExtractor(){_featureName = FEATURE_NAME; _descriptorName = DESCRIPTOR_NAME;};
        vector<KeyPoint> extractFeatures(Mat img);
        Mat computeDescriptors(const Mat img, vector<KeyPoint> &keyPoints1);
    private:
        string _featureName;
        string _descriptorName;
        
    };
}

#endif /* FeatureCandidateExtractor_h */
