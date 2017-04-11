//
//  FeatureDetector.cpp
//  VisualSLAM
//
//  Created by Rong Yuan on 2/28/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#include "FeatureExtractor.h"

vector<KeyPoint> FeatureExtractor::extractFeatures(Mat img, FeatureSet& featureSet)
{
    vector<KeyPoint> keypoints;
    Ptr<SURF> detector = SURF::create();
    detector->detect(img, keypoints);
    // remove existing feature points
    featureSet.clear();
    IdGenerator* idGenerator = IdGenerator::createInstance();
    for(int i = 0; i < keypoints.size(); i++)
    {
        featureSet.addFeature(keypoints[i], idGenerator->next());
    }
    return keypoints;
}

vector<KeyPoint> FeatureExtractor::reextractFeatures(Mat img, FeatureSet& featureSet)
{
    vector<KeyPoint> keypoints;
    Ptr<SURF> detector = SURF::create();
    detector->detect(img, keypoints);
    // append newly extracted feature points back to existing feature set
    map<double, set<double>> existingPointSet;
    vector<KeyPoint> existingPoints = featureSet.getFeaturePoints();
    for(int i = 0; i < existingPoints.size(); i++)
        existingPointSet[existingPoints[i].pt.x].insert(existingPoints[i].pt.y);
    IdGenerator* idGenerator = IdGenerator::createInstance();
    for(int i = 0; i < keypoints.size(); i++)
    {
        if(existingPointSet.count(keypoints[i].pt.x) &&
           existingPointSet[keypoints[i].pt.x].count(keypoints[i].pt.y))
            continue;
        featureSet.addFeature(keypoints[i], idGenerator->next());
    }
    return keypoints;
}
//vector<KeyPoint> FeatureExtractor::reextractFeatures(Mat img, FeatureSet& featureSet)
//{
//    vector<Point2f> newPoints;
//    vector<KeyPoint> keypoints;
//    int fast_threshold = 20;
//    bool nonmaxSuppression = true;
//    FAST(img, keypoints, fast_threshold, nonmaxSuppression);
//    KeyPoint::convert(keypoints, newPoints, vector<int>());
//    // append newly extracted feature points back to existing feature set
//    map<double, set<double>> existingPointSet;
//    vector<Point2f> existingPoints = featureSet.getFeaturePoints();
//    for(int i = 0; i < existingPoints.size(); i++)
//        existingPointSet[existingPoints[i].x].insert(existingPoints[i].y);
//    IdGenerator* idGenerator = IdGenerator::createInstance();
//    for(int i = 0; i < newPoints.size(); i++)
//    {
//        if(existingPointSet.count(newPoints[i].x) &&
//           existingPointSet[newPoints[i].x].count(newPoints[i].y))
//            continue;
//        featureSet.addFeature(newPoints[i], idGenerator->next());
//    }
//    return keypoints;
//}

