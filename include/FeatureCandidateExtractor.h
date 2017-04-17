//
//  FeatureCandidateDetector.hpp
//  VisualSLAM
//
//  Created by Rong Yuan on 4/14/17.
//  Copyright © 2017 Hongyi Fan. All rights reserved.
//

#ifndef FeatureCandidateExtractor_h
#define FeatureCandidateExtractor_h

#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "Feature.h"
#include "Factory.h"
#include "SystemParameters.h"
#include <iostream>
#include <stdio.h>
#include <set>
#include <map>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

class FeatureCandidateExtractor {
public:
    FeatureCandidateExtractor(){_featureName = FEATURE_NAME; _descriptorName = DESCRIPTOR_NAME;};
    vector<KeyPoint> extractFeatures(Mat img);
    Mat computeDescriptors(const Mat img, vector<KeyPoint> &keyPoints1);
    //vector<KeyPoint> reextractFeatures(Mat img_1, FeatureSet& featureSet);
private:
    string _featureName;
    string _descriptorName;
    
};
#endif /* FeatureCandidateExtractor_h */
