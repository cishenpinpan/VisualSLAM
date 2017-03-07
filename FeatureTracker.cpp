//
//  FeatureTracker.cpp
//  VisualSLAM
//
//  Created by Rong Yuan on 2/28/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#include "FeatureTracker.h"

bool LEFT_CAMERA = true, RIGHT_CAMERA = false;

// general
vector<uchar> FeatureTracker::track(Mat img1, Mat img2,
                                    FeatureSet& featureSet1, FeatureSet& featureSet2, bool stereo)
{
    map<int, int> indexBook;
    //this function automatically gets rid of points for which tracking fails
    
    vector<float> err;
    vector<uchar> status;
    Size winSize=Size(21,21);
    TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
    
    calcOpticalFlowPyrLK(img1, img2, featureSet1.getFeaturePoints(), featureSet2.getFeaturePoints(),
                         status, err, winSize, 3, termcrit, 0, 0.001);
    
    // sync feature id in feature set 2
    featureSet2.setIds(featureSet1.getIds());
    
    return status;
}

// stereo track
void FeatureTracker::track(View *v)
{
    // return track(v->getImgs()[0], v->getImgs()[1], v->getLeftFeatures(), v->getRightFeatures(), v->getFids(), true);
    ;
}

// temporal views
void FeatureTracker::track(View *v1, View *v2, bool camera = LEFT_CAMERA)
{
//    // assume v1 is stereo matched
//    // 1. temporal match in selected camera coordinate (left/right)
//    vector<Point2f> points1 = camera ? v1->getLeftFeatures() : v1->getRightFeatures();
//    vector<Point2f> points2 = camera ? v2->getLeftFeatures() : v2->getRightFeatures();
//    map<int, int> fids = v1->getFids(), indexBook;
//    indexBook = track(v1->getImgs()[0], v2->getImgs()[0], points1, points2, fids, false);
//    v1->setLeftFeatures(points1);
//    v1->setFids(map<int, int>(fids));
//    v2->setLeftFeatures(points2);
//    v2->setFids(map<int, int>(fids));
//    // also update right image of v1
//    vector<Point2f> oldRightPointsV1 = v1->getRightFeatures();
//    vector<Point2f> newRightPointsV1(v1->getLeftFeatures().size(), Point2f(0.0, 0.0));
//    for(int i = 0; i < oldRightPointsV1.size(); i++)
//    {
//        Point2f oldPoint = oldRightPointsV1[i];
//        if(indexBook.count(i) == 0)
//            continue;
//        int newIndex = indexBook[i];
//        newRightPointsV1[newIndex] = Point2f(oldPoint);
//    }
//    v1->setRightFeatures(newRightPointsV1);
//    
//    // 2. stereo match between left and right for v2
//    indexBook = track(v2);
//    
//    // 3. update v1 according to fids from v2
//    vector<Point2f> oldPointsLeftV1 = v1->getLeftFeatures(), oldPointsRightV1 = v1->getRightFeatures();
//    vector<Point2f> newPoints1(indexBook.size(), Point2f(0.0, 0.0)), newPoints2(indexBook.size(), Point2f(0.0, 0.0));
//    map<int, int> newFids;
//    map<int, int> targetFids = v2->getFids();
//    for(int i = 0; i < oldPointsLeftV1.size(); i++)
//    {
//        if(indexBook.count(i) == 0)
//            continue;
//        int newIndex = indexBook[i];
//        newPoints1[newIndex] = oldPointsLeftV1[i];
//        newPoints2[newIndex] = oldPointsRightV1[i];
//    }
//    v1->setLeftFeatures(newPoints1);
//    v1->setRightFeatures(newPoints2);
//    v1->setFids(v2->getFids());
    ;
}
