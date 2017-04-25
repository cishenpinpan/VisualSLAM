//
//  FeatureDetector.cpp
//  VisualSLAM
//
//  Created by Rong Yuan on 2/28/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#include "FeatureExtractor.h"
#include "Canvas.h"

vector<KeyPoint> FeatureExtractor::extractFeatures(Mat img, FeatureSet& featureSet)
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
    // remove existing feature points
    featureSet.clear();
    IdGenerator* idGenerator = IdGenerator::createInstance();
    vector<KeyPoint> newPoints;
    vector<long> newIds;
    for(int i = 0; i < keypoints.size(); i++)
    {
        newPoints.push_back(keypoints[i]);
        newIds.push_back(idGenerator->next());
    }
    featureSet.setFeaturePoints(newPoints);
    featureSet.setIds(newIds);
    return keypoints;
}

vector<KeyPoint> FeatureExtractor::reextractFeatures(Mat img, FeatureSet& featureSet)
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
 
    // append newly extracted feature points back to existing feature set
    map<double, set<double>> existingPointSet;
    vector<KeyPoint> existingPoints = featureSet.getFeaturePoints();
    //	Canvas canvas;
    //	canvas.drawKeyPoints(img,keypoints,"Change threshold");
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

