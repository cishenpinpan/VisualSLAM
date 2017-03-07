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
#include "camera.hpp"

using namespace std;
using namespace cv;

class Canvas
{
public:
    void drawFeatureMatches(Mat img1, Mat img2, vector<Point2f> points1, vector<Point2f> points2);
};

#endif /* Canvas_h */
