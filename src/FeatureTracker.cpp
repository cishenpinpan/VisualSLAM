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
    vector<Point2f> points1 = Converter::keyPointsToPoint2fs(keyPoints1), trackings;
    vector<float> err;
    vector<uchar> status;
    Size winSize=Size(21,21);
    TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
    
    // KLT Tracker
    calcOpticalFlowPyrLK(img1, img2, points1, trackings, status, err, winSize, 3, termcrit, 0, 0.001);
    
    // compute corners in img2
    Ptr<SURF> detector = SURF::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");
    // compute descriptors of candidates
    Mat descriptor1, descriptors2;
    Ptr<SURF> extractor = SURF::create();
    extractor->compute(img1, keyPoints1, descriptor1);
    extractor->detectAndCompute(img2, Mat(), keyPoints2, descriptors2);
    map<int, map<int, vector<KeyPoint> > > cornerMap;
    for(KeyPoint kp : keyPoints2)
    {
        cornerMap[int(kp.pt.x)][int(kp.pt.y)].push_back(kp);
    }
    // for each feature track, search in a neighborhood for real corners
    vector<KeyPoint> newKeyPoints1, newKeyPoints2;
    vector<long> prevIds = featureSet1.getIds(), newIds;
    
    for(int i = 0; i < trackings.size(); i++)
    {
        KeyPoint p1 = keyPoints1[i];
        Point2f t = trackings[i];
        long id = prevIds[i];
        if(status[i] == 0)
            continue;
        if(t.x <= 0 || t.x >= 1281 || t.y <= 0 || t.y >= 376)
            continue;
        // search appearance of corners in neighborhood
        KeyPoint bestMatch;
        float minDist = -1, secondMinDist = -1;
        float lowerBound = 100;
        for(int x = t.x - 3; x <= t.x + 3; x++)
        {
            for(int y = t.y - 3; y <= t.y + 3; y++)
            {
                if(x <= 0 || x >= 1281 || y <= 0 || y >= 376)
                    continue;
                // go through all keypoints
                if(cornerMap.count(x) == 0 || cornerMap[x].count(y) == 0)
                    continue;
                for(KeyPoint p2 : cornerMap[x][y])
                {
                    // keypoint found!
                    // see how good this match is (distance of descriptors)
                    // record only the best match
                    vector<vector<DMatch>> match;
                    Mat tempDes1, tempDes2;
                    vector<KeyPoint> tempKps1, tempKps2;
                    tempKps1.push_back(p1);
                    tempKps2.push_back(p2);
                    extractor->compute(img1, tempKps1, tempDes1);
                    extractor->compute(img2, tempKps2, tempDes2);
                    matcher->knnMatch(tempDes1, tempDes2, match, 1);
                    lowerBound = match[0][0].distance < lowerBound ? match[0][0].distance : lowerBound;
                    if(minDist == - 1 || match[0][0].distance < minDist)
                    {
                        secondMinDist = minDist;
                        bestMatch = p2;
                        minDist = match[0][0].distance;
                    
                    }
//                    Canvas canvas;
//                    vector<Point2f> ps1, ps2, track;
//                    KeyPoint::convert(tempKps1, ps1);
//                    KeyPoint::convert(tempKps2, ps2);
//                    track.push_back(trackings[i]);
//                    canvas.drawFeatureMatches(img1, img2, ps1, track);
//                    canvas.drawFeatureMatches(img1, img2, ps1, ps2);
                }
            }
        }
        if(minDist != -1 && minDist < 0.2)
        {
            newKeyPoints1.push_back(p1);
            newKeyPoints2.push_back(bestMatch);
            newIds.push_back(id);
        }
    }
    // update features and ids
    featureSet1.setFeaturePoints(newKeyPoints1);
    featureSet1.setIds(newIds);
    featureSet2.setFeaturePoints(newKeyPoints2);
    featureSet2.setIds(newIds);
    
//    Canvas canvas;
//    for(int i = 0; i < 100; i++)
//    {
//        vector<Point2f> ps1, ps2;
//        vector<KeyPoint> kps1, kps2;
//        kps1.push_back(featureSet1.getFeaturePoints()[i]);
//        kps2.push_back(featureSet2.getFeaturePoints()[i]);
//        KeyPoint::convert(kps1, ps1);
//        KeyPoint::convert(kps2, ps2);
//        canvas.drawFeatureMatches(img1, img2, ps1, ps2);
//    }
    

    Canvas canvas;
    vector<Point2f> ps1, ps2;
    KeyPoint::convert(featureSet1.getFeaturePoints(), ps1);
    KeyPoint::convert(featureSet2.getFeaturePoints(), ps2);
    // canvas.drawFeatureMatches(img1, img2, ps1, ps2);
    
    return Mat::ones(featureSet1.size(), 1, CV_64F);
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
        if(minDist == -1 || minDist > 0.4)
            continue;
        newKeyPoints.push_back(bestKp);
        newIds.push_back(it->first);
    }
    v->getLeftFeatureSet().setFeaturePoints(newKeyPoints);
    v->getLeftFeatureSet().setIds(newIds);
    Canvas canvas;
    vector<Point2f> ps1, ps2;
    for(long id: newIds)
    {
        KeyPoint tmp = prevView->getLeftFeatureSet().getFeatureById(id).getPoint();
        ps1.push_back(Point2f(tmp.pt.x, tmp.pt.y));
    }
    KeyPoint::convert(v->getLeftFeatureSet().getFeaturePoints(), ps2);
    canvas.drawFeatureMatches(prevView->getImgs()[0], img, ps1, ps2);
    Mat status = Mat::ones(v->getLeftFeatureSet().size(), 1, CV_64F);
    return status.clone();
}
