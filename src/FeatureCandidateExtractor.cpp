//
//  FeatureCandidateDetector.cpp
//  VisualSLAM
//
//  Created by Hongyi Fan on 4/14/17.
//  Copyright © 2017 Hongyi Fan. All rights reserved.
//

#include "FeatureCandidateExtractor.h"

vector<KeyPoint> FeatureCandidateExtractor::extractFeatures(Mat img)
{
    vector<KeyPoint> keypoints;
    
    if (_featureName == "SURF")
    {
        Ptr<SURF> detector = SURF::create();
        //detector->setNOctaves(6);
        detector->setNOctaveLayers(10);
        detector->detect(img, keypoints);
        
    }
    else if (_featureName == "ORB")
    {
        Ptr<ORB> detector = ORB::create();
        detector->setScaleFactor(2);
        detector->setMaxFeatures(3500);
        detector->setNLevels(6);
        detector->detect(img, keypoints);
    }
    else
    {
        std::cout << "Not Supported" << std::endl;
    }
    
    return keypoints;
}

Mat FeatureCandidateExtractor::computeDescriptors(const Mat img, vector<KeyPoint> &keyPoints1)
{
    Mat descriptors;
    if (_featureName == "SURF")
    {
        Ptr<SURF> detector = SURF::create();
        //detector->setNOctaves(6);
        detector->setNOctaveLayers(10);
        detector->compute(img, keyPoints1,descriptors);
        
    }
    else if (_featureName == "ORB")
    {
        Ptr<ORB> detector = ORB::create();
        detector->setScaleFactor(2);
        detector->setMaxFeatures(3500);
        detector->setNLevels(6);
        cout << keyPoints1.size() << endl;
        detector->compute(img, keyPoints1,descriptors);

    }
    else
    {
        std::cout << "Not Supported" << std::endl;
    }
    return descriptors.clone();
}
