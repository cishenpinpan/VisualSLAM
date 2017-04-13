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


void Canvas::drawSingleImage(Mat img1)
{
	namedWindow("Single image",WINDOW_AUTOSIZE);
	imshow("Single image", img1);
	waitKey(0);
}

void Canvas::drawKeyPoints(Mat img1, vector<Point2f> points1, string windowName)
{
	Mat canvas;
	vector<KeyPoint> Keypoints1;
	for (int i = 0; i < points1.size(); i++)
	{
		Keypoints1.push_back(KeyPoint(points1[i],1.f));
	}
	drawKeypoints(img1,Keypoints1,canvas);

	namedWindow(windowName, WINDOW_AUTOSIZE);
	imshow(windowName, canvas);
	waitKey(0);

}

void Canvas::drawKeyPoints(Mat img1, vector<KeyPoint> points1, string windowName)
{
	Mat canvas;
	drawKeypoints(img1, points1, canvas);

	namedWindow(windowName, WINDOW_AUTOSIZE);
	imshow(windowName, canvas);
	waitKey(0);
}



void Canvas::drawTrackingPathEveryOtherK(Mat img, vector<KeyPoint> keyPoints1, vector<KeyPoint> keyPoints2, int numOfDrawing)
{
	vector<KeyPoint> tempKeyPoints1, tempKeyPoints2;
	for (int i = 0; i < keyPoints1.size(); i += numOfDrawing)
	{
		tempKeyPoints1.push_back(keyPoints1[i]);
		tempKeyPoints2.push_back(keyPoints2[i]);
	}

	drawTrackingPath(img, tempKeyPoints1, tempKeyPoints2);
}



void Canvas::drawLine( Mat img, Point2f start, Point2f end )
{
	int thickness = 0.5;
	int lineType = LINE_8;
	line( img,start,end,Scalar( 255, 0, 0 ),thickness,lineType );
}

void Canvas::drawTrackingPath(Mat img, vector<Point2f> keyPoints1, vector<Point2f> keyPoints2)
{
	for (int i = 0; i < keyPoints1.size(); i++)
	{
		drawLine(img, keyPoints1[i], keyPoints2[i]);
	}

	vector<KeyPoint> TempKeypoints1;
	for (int i = 0; i < keyPoints1.size(); i++)
	{
		TempKeypoints1.push_back(KeyPoint(keyPoints1[i],1.f));
	}
	drawKeypoints(img,TempKeypoints1,img);
	namedWindow("Tracking Path", WINDOW_AUTOSIZE);
	imshow("Tracking Path", img);
	waitKey(0);
}

void Canvas::drawTrackingPath(Mat img, vector<KeyPoint> keyPoints1, vector<KeyPoint> keyPoints2)
{
	vector<Point2f> tempPoints1, tempPoints2;
	KeyPoint::convert(keyPoints1, tempPoints1);
	KeyPoint::convert(keyPoints2, tempPoints2);
	for (int i = 0; i < keyPoints1.size(); i++)
	{

		drawLine(img, tempPoints1[i], tempPoints2[i]);
	}
	drawKeypoints(img, keyPoints1, img);
	namedWindow("Tracking Path", WINDOW_AUTOSIZE);
	imshow("Tracking Path", img);
	waitKey(0);
}













