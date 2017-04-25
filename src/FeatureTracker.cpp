//
//  FeatureTracker.cpp
//  VisualSLAM
//
//  Created by Rong Yuan on 2/28/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#include "FeatureTracker.h"

bool LEFT_CAMERA = true, RIGHT_CAMERA = false;

void FeatureTracker::kltTrack(const Mat img1, const Mat img2, FeatureSet& featureSet1, FeatureSet& featureSet2, bool stereo)
{
    Canvas canvas;
    
    //Do KLT track between two images
    vector<KeyPoint> keyPoints1 = featureSet1.getFeaturePoints();
    vector<Point2f> points1 = Converter::keyPointsToPoint2fs(keyPoints1), trackings;
    vector<float> err;
    vector<uchar> status;
    Size winSize=Size(31,31);
    TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
    calcOpticalFlowPyrLK(img1,img2,points1,trackings, status, err, winSize, 3, termcrit, 0, 0.001);
    
    // build FeatureSet2
    vector<KeyPoint> keyPoints2;
    for (int i = 0; i < trackings.size(); i++)
        keyPoints2.push_back(KeyPoint(trackings[i],1.0f));
    
    featureSet2.setFeaturePoints(keyPoints2);
    featureSet2.setIds(featureSet1.getIds());
    
    //Remove the bad tracking according to image size and really large motion
    for (int i = 0; i < status.size(); i++)
    {
        Point2f t = trackings[i];
        if(t.x <= 0 || t.x >= 1281 || t.y <= 0 || t.y >= 376)
        {
            status[i] = (uchar)0;
        }
    }
    //Remove the bad tracking according to status
    vector<KeyPoint> newFeatures1;
    vector<KeyPoint> newFeatures2;
    vector<long> currIds = featureSet1.getIds(), newIds;
    for (int i = 0; i < status.size() ;i++)
    {
        Feature feature1 = featureSet1.getFeaturePoints()[i];
        Feature feature2 = featureSet2.getFeaturePoints()[i];
        long id = currIds[i];
        if (status[i] == 1)
        {
            
            newFeatures1.push_back(feature1.getPoint());
            newFeatures2.push_back(feature2.getPoint());
            newIds.push_back(id);
        }
    }
    
    // featureSet1.setFeaturePoints(newFeatures1);
    // featureSet1.setIds(newIds);
    featureSet2.setFeaturePoints(newFeatures2);
    featureSet2.setIds(newIds);
}

void FeatureTracker::refineTrackedFeatures
            (Mat img1, Mat img2, FeatureSet& featureSet1, FeatureSet& featureSet2, bool stereo)
{
    Canvas canvas;
    clock_t t0, t1, t2, t3, t4, t5;
    t0 = clock();
    vector<KeyPoint> keyPointsFirstView;
    vector<KeyPoint> trackedPointsLastView = featureSet2.getFeaturePoints();
    for (int j = 0; j < featureSet2.size(); j++)
    {
        keyPointsFirstView.push_back(featureSet1.getFeatureById(featureSet2.getIds()[j]).getPoint());
    }
    t1 = clock();
    // Create feature extractor object
    FeatureCandidateExtractor extractor;
    //Compute descriptors from the last view
    vector<KeyPoint> keyPointsLastView = extractor.extractFeatures(img2);
    t2 = clock();
    Mat descriptorsLastView = extractor.computeDescriptors(img2, keyPointsLastView);
//    extractor.computeDescriptors(img1, keyPointsLastView);
    
    // build a coordinate look-up to be used in searching keypoints in a patch
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");
    map<int, map<int, vector<pair<KeyPoint, Mat> > > > cornerMap;
    for(int i = 0; i < keyPointsLastView.size(); i++)
    {
        KeyPoint kp = keyPointsLastView[i];
        cornerMap[int(kp.pt.x)][int(kp.pt.y)].push_back(pair<KeyPoint, Mat>(kp, descriptorsLastView.row(i)));
    }
    t3 = clock();
    vector<KeyPoint> newKeyPoints1, newKeyPoints2;
    vector<long> prevIds = featureSet2.getIds(), newIds;
    int winSize = 2;
    
    //For each feature track Search keypoints in the neighborhood of tracked points in last view
    for(int i = 0; i < trackedPointsLastView.size(); i++)
    {
        KeyPoint p1 = keyPointsFirstView[i];
        KeyPoint t = trackedPointsLastView[i];
        long id = prevIds[i];
        
        // compute descriptor for p1
        vector<KeyPoint> tmp;
        tmp.push_back(p1);
        Mat descP1 = extractor.computeDescriptors(img1, tmp);
        
        // out of bound check
        if(t.pt.x <= 0 || t.pt.x >= 1281 || t.pt.y <= 0 || t.pt.y >= 376)
            continue;
        
        // search for the appearances of corners in neighborhood
        pair<KeyPoint, Mat> bestMatch;
        float minDist = -1, secondMinDist = -1;
        for(int x = t.pt.x - winSize; x <= t.pt.x + winSize; x++)
        {
            for(int y = t.pt.y - winSize; y <= t.pt.y + winSize; y++)
            {
                if(x <= 0 || x >= 1281 || y <= 0 || y >= 376)
                    continue;
                // go through all keypoints
                if(cornerMap.count(x) == 0 || cornerMap[x].count(y) == 0)
                {
                    continue;
                }
                //std::cout << "Size:" << cornerMap[x][y].size() << std::endl;
                for(pair<KeyPoint, Mat> p2Pair : cornerMap[x][y])
                {
                    // keypoint found!
                    // see how good this match is (distance of descriptors)
                    // record only the best match
                    vector<vector<DMatch>> match;
                    Mat descP2 = p2Pair.second;
                    vector<KeyPoint> tempKps1, tempKps2;
                    tempKps1.push_back(p1);
                    tempKps2.push_back(p2Pair.first);
                    matcher->knnMatch(descP1, descP2, match, 1);
                    if(minDist == - 1 || match[0][0].distance < minDist)
                    {
                        secondMinDist = minDist;
                        bestMatch = p2Pair;
                        minDist = match[0][0].distance;
                    }
                }
            }
        }
        if(minDist != -1 && (minDist < 0.8 * secondMinDist || secondMinDist == -1))
        {
            newKeyPoints1.push_back(p1);
            newKeyPoints2.push_back(bestMatch.first);
            newIds.push_back(id);
        }
        
    }
    t4 = clock();
    // featureSet1.setFeaturePoints(newKeyPoints1);
    // featureSet1.setIds(newIds);
    featureSet2.setFeaturePoints(newKeyPoints2);
    featureSet2.setIds(newIds);
    t5 = clock();
//    cout << (float(t1) - float(t0)) / CLOCKS_PER_SEC << endl;
//    cout << (float(t2) - float(t1)) / CLOCKS_PER_SEC << endl;
//    cout << (float(t3) - float(t2)) / CLOCKS_PER_SEC << endl;
//    cout << (float(t4) - float(t3)) / CLOCKS_PER_SEC << endl;
//    cout << (float(t5) - float(t4)) / CLOCKS_PER_SEC << endl;
//    cout << ".........." << endl;
}

// Assume features are extracted in the first view
// Track these features all the way to the last view,
// and match the nearest keypoint neighbor
void FeatureTracker::trackAndMatch(vector<View*> views)
{
    for(int i = 0; i < views.size() - 1; i++)
    {
        View *v1 = views[i], *v2 = views[i + 1];
        kltTrack(v1->getImgs()[0], v2->getImgs()[0], v1->getLeftFeatureSet(), v2->getLeftFeatureSet(), false);
    }
    // refine (Lowe's)
    View *v1 = views[0], *v2 = views.back();
    refineTrackedFeatures(v1->getImgs()[0], v2->getImgs()[0], v1->getLeftFeatureSet(), v2->getLeftFeatureSet(), false);
}
void FeatureTracker::track(vector<View*> views)
{
    for(int i = 0; i < views.size() - 1; i++)
    {
        View *v1 = views[i], *v2 = views[i + 1];
        kltTrack(v1->getImgs()[0], v2->getImgs()[0], v1->getLeftFeatureSet(), v2->getLeftFeatureSet(), false);
    }
}

void FeatureTracker::trackAndMatch(View *v)
{
    if(v->isStereo())
    {
        kltTrack(v->getImgs()[0], v->getImgs()[1], v->getLeftFeatureSet(), v->getRightFeatureSet(), false);
        refineTrackedFeatures(v->getImgs()[0], v->getImgs()[1], v->getLeftFeatureSet(), v->getRightFeatureSet(), false);
    }
}

void FeatureTracker::track(View *v)
{
    if(v->isStereo())
    {
        kltTrack(v->getImgs()[0], v->getImgs()[1], v->getLeftFeatureSet(), v->getRightFeatureSet(), false);
    }
}


