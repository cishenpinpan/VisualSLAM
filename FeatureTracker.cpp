//
//  FeatureTracker.cpp
//  VisualSLAM
//
//  Created by Rong Yuan on 2/28/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#include "FeatureTracker.h"

bool LEFT_CAMERA = true, RIGHT_CAMERA = false;

Mat FeatureTracker::track(Mat img1, Mat img2,
                          FeatureSet& featureSet1, FeatureSet& featureSet2, bool stereo)
{
    vector<KeyPoint> keyPoints1 = featureSet1.getFeaturePoints(), keyPoints2;
    
    // compute corners in img2
    Ptr<SURF> detector = SURF::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");
    // compute descriptors of candidates
    Mat descriptor1, descriptors2;
    Ptr<SURF> extractor = SURF::create();
    extractor->compute(img1, keyPoints1, descriptor1);
    extractor->detectAndCompute(img2, Mat(), keyPoints2, descriptors2);
    vector<vector<DMatch>> matches;
    matcher->knnMatch( descriptor1, descriptors2, matches, 2 );
    vector<KeyPoint> newKeyPoints1, newKeyPoints2;
    vector<long> prevIds = featureSet1.getIds(), newIds;
    
    for(int i = 0; i < matches.size(); i++)
    {
        KeyPoint kp1 = keyPoints1[i], kp2 = keyPoints2[matches[i][0].trainIdx];
        // Lowe's
        if(matches[i][0].distance < 0.7 * matches[i][1].distance)
        {
            newKeyPoints1.push_back(kp1);
            newKeyPoints2.push_back(kp2);
            newIds.push_back(prevIds[i]);
        }
    }
    
    featureSet1.setFeaturePoints(newKeyPoints1);
    featureSet1.setIds(newIds);
    featureSet2.setFeaturePoints(newKeyPoints2);
    featureSet2.setIds(newIds);
    
    
    
    Mat status = Mat::ones(featureSet1.size(), 1, CV_64F);
    return status.clone();
}
// track from landmarks
Mat FeatureTracker::searchLandmarks(View* v, map<long, Landmark> landmarkBook, View* prevView)
{
    // retrieve image information
    const Mat img = v->getImgs()[0];
    vector<KeyPoint> keyPoints;
    // compute corners
    Ptr<SURF> extractor = SURF::create();
    Mat descriptors;
    extractor->detectAndCompute(img, Mat(), keyPoints, descriptors);
    map<int, map<int, vector<KeyPoint> > > cornerMap;
    for(KeyPoint kp : keyPoints)
    {
        cornerMap[int(kp.pt.x)][int(kp.pt.y)].push_back(kp);
    }
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");
    // traverse through all landmarks
    // for each one, find its match in a projected neighborhood
    vector<KeyPoint> newKeyPoints;
    vector<long> newIds;
    for(map<long, Landmark>::iterator it = landmarkBook.begin(); it != landmarkBook.end(); it++)
    {
        // project the landmark onto the image
        const Landmark l = it->second;
        Mat L = Mat::zeros(3, 1, CV_64F);
        L.at<double>(0, 0) = l.point3d.x;
        L.at<double>(1, 0) = l.point3d.y;
        L.at<double>(2, 0) = l.point3d.z;
        const Mat p = v->getPose();
        const Mat R = p.colRange(0, 3).rowRange(0, 3);
        const Mat t = p.col(3).rowRange(0, 3);
        const Mat projected3d = R.inv() * L - R.inv() * t;
        const Mat projected2d = CameraParameters::getIntrinsic() * (projected3d / projected3d.at<double>(2, 0));
        
        const Point2d projected(projected2d.at<double>(0, 0), projected2d.at<double>(1, 0));
        // ignore out-of-range points
        if(projected.x < 0 || projected.x > 1281 || projected.y < 0 || projected.y > 376)
            continue;
        Mat outputImg1, outputImg2;
        
        // waitKey(0);
        // search in neighborhood
        KeyPoint bestKp;
        float minDist = -1;
        for(int x = projected.x - 20; x <= projected.x + 20; x++)
        {
            if(x < 0 || x > 1281)
                continue;
            for(int y = projected.y - 20; y <= projected.y + 20; y++)
            {
                if(y < 0 || y > 376)
                    continue;
                // go through all keypoints
                if(cornerMap.count(x) == 0 || cornerMap[x].count(y) == 0)
                    continue;
                for(KeyPoint kp : cornerMap[x][y])
                {
                    // keypoint found!
                    vector<vector<DMatch>> match;
                    Mat tempDes;
                    vector<KeyPoint> tempKps;
                    tempKps.push_back(kp);
                    extractor->compute(img, tempKps, tempDes);
                    matcher->knnMatch(l.descriptor, tempDes, match, 1);
                    if(minDist == - 1 || match[0][0].distance < minDist)
                    {
                        bestKp = kp;
                        minDist = match[0][0].distance;
                    }
                }
            }
        }
        if(minDist == -1 || minDist > 0.2)
            continue;
        Canvas canvas;
        vector<Point2f> ps1 = {Point2f(prevView->getLeftFeatureSet().getFeatureById(it->first).getPoint().pt.x, prevView->getLeftFeatureSet().getFeatureById(it->first).getPoint().pt.y)}, ps2;
        vector<KeyPoint> tempKps;
        tempKps.push_back(bestKp);
        KeyPoint::convert(tempKps, ps2);
        canvas.drawFeatureMatches(prevView->getImgs()[0], img, ps1, ps2);
        newKeyPoints.push_back(bestKp);
        newIds.push_back(it->first);
        
    }
    v->getLeftFeatureSet().setFeaturePoints(newKeyPoints);
    v->getLeftFeatureSet().setIds(newIds);
    
    Mat status = Mat::ones(v->getLeftFeatureSet().size(), 1, CV_64F);
    return status.clone();
}
