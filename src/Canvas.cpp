//
//  Canvas.cpp
//  VisualSLAM
//
//  Created by Rong Yuan on 2/28/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#include "Canvas.h"

void Canvas::drawFeatureMatches(Mat img1, Mat img2, vector<Point2f> points1,vector<Point2f> points2)
{
    Mat canvas;
    vector<DMatch> matches;
    vector<KeyPoint> currKeypoints, nextKeypoints;
    for(int k = 0; k < points1.size(); k++)
    {
        matches.push_back(DMatch(k, k, 0));
        currKeypoints.push_back(KeyPoint(points1[k], 1.f));
        nextKeypoints.push_back(KeyPoint(points2[k], 1.f));
    }
    drawMatches(img1, currKeypoints, img2, nextKeypoints, matches, canvas);
    
    namedWindow("Display window", WINDOW_AUTOSIZE);
    imshow("Display window", canvas);
    waitKey(0);
}

