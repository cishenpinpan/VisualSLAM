//
//  ImageReader.h
//  VisualSLAM
//
//  Created by Rong Yuan on 2/22/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//
#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>
#include <list>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

#ifndef ViewReader_h
#define ViewReader_h

class ViewReader
{
public:
    ViewReader(string _dataset, string _track, bool _stereo);
    // get next view
    vector<Mat> next();
    string getTrack(){return track;}
    
private:
    string dataset;
    string track;
    bool stereo;
    int index;
};

#endif /* ViewReader_h */
