//
//  Canvas.hpp
//  VisualSLAM
//
//  Created by Rong Yuan on 2/28/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#ifndef Canvas_h
#define Canvas_h

#include <stdio.h>
#include <vector>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace std;
using namespace cv;

class Canvas
{
public:
    void drawFeatureMatches(Mat img1, Mat img2, vector<Point2f> points1, vector<Point2f> points2);
	void drawSingleImage(Mat img1);

	void drawKeyPoints(Mat img1, vector<Point2f> Keypoints1, string windowName);
	void drawKeyPoints(Mat img1, vector<KeyPoint> Keypoints1, string windowName);


	void drawTrackingPathEveryOtherK(Mat img, vector<KeyPoint> keyPoints1, vector<KeyPoint> keyPoints2, int numOfDrawing);
	void drawLine(Mat img, Point2f start, Point2f end);
	void drawTrackingPath(Mat img, vector<KeyPoint> keyPoints1, vector<KeyPoint> keyPoints2);
	void drawTrackingPath(Mat img, vector<Point2f> keyPoints1, vector<Point2f> keyPoints2);
};

#endif /* Canvas_h */
