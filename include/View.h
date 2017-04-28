//
//  Frame.h
//  VisualSLAM
//
//  Created by Rong Yuan on 2/24/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#ifndef View_h
#define View_h
#include "opencv2/features2d.hpp"
#include "CameraParameters.h"
#include "Feature.h"
#include "Factory.h"
#include <iostream>
#include <vector>
#include <map>

using namespace std;
using namespace cv;

class View
{
public:
    View(View *v, int _time);
    ~View();
    View(const vector<Mat> _imgs, int _time);
    View(const vector<Mat> _imgs, const FeatureSet _leftFeatureSet, int _time);
    long getId();
    vector<Mat> getImgs();
    void setImgs(const vector<Mat> _imgs);
    void deleteImgs();
    FeatureSet& getLeftFeatureSet();
    FeatureSet& getRightFeatureSet();
    Feature getLeftFeatureById(long id);
    Feature getRightFeatureById(long id);
    void removeLeftFeatureById(long id);
    void removeRightFeatureById(long id);
    void setLeftFeatures(const FeatureSet _leftFeatureSet);
    void setRightFeatures(const FeatureSet _rightFeatureSet);
    void setIdBook(const map<long, int> _idBook);
    Mat getPose();
    void setPose(const Mat _pose);
    void setPose(const Mat _R, const Mat _T);
    Mat getR();
    Mat getT();
    void setKeyView(){keyView = true;}
    bool isKeyView(){return keyView;}
    bool isStereo(){return stereo;}
    int getTime(){return time;}
private:
    long id;
    bool keyView;
    int time;
    vector<Mat> imgs;
    FeatureSet leftFeatureSet;
    FeatureSet rightFeatureSet;
    Mat pose;
    bool stereo;
};

#endif /* View_h */
