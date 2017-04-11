//
//  ImageReader.cpp
//  VisualSLAM
//
//  Created by Rong Yuan on 2/22/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#include "ViewReader.h"

ViewReader::ViewReader(string _dataset, string _track, bool _stereo)
{
    dataset = _dataset;
    track = _track;
    stereo = _stereo;
    index = 0;
}

vector<Mat> ViewReader::next()
{
    vector<Mat> images;
    Mat leftImage;
    string fileDir = IMAGE_DIR + track + "/";
    fileDir = fileDir + "image_0/" + string(6 - to_string(index).size(), '0') + to_string(index);
    fileDir = fileDir + ".png";
    
    leftImage = imread(fileDir, CV_LOAD_IMAGE_GRAYSCALE);   // Read the file
    images.push_back(leftImage);
    if(stereo)
    {
        Mat rightImage;
        string fileDir = IMAGE_DIR + track + "/";
        fileDir = fileDir + "image_1/" + string(6 - to_string(index).size(), '0') + to_string(index);
        fileDir = fileDir + ".png";
        rightImage = imread(fileDir, CV_LOAD_IMAGE_GRAYSCALE);   // Read the file
        images.push_back(rightImage);
    }
    ++index;
    return images;
}
