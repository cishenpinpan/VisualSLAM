//
//  Frame.cpp
//  VisualSLAM
//
//  Created by Rong Yuan on 2/24/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#include "View.h"

View::View(const vector<Mat> _imgs, int _time)
{
    imgs = vector<Mat>();
    for(int i = 0; i < _imgs.size(); i++)
    {
        imgs.push_back(_imgs[i].clone());
    }
    stereo = imgs.size() == 2 ? true : false;
    IdGenerator *idGenerator = IdGenerator::createInstance();
    id = idGenerator->next();
    time = _time;
    keyView = false;
}

View::View(const vector<Mat> _imgs, const FeatureSet _leftFeatureSet, int _time)
{
    imgs = vector<Mat>();
    for(int i = 0; i < _imgs.size(); i++)
    {
        imgs.push_back(_imgs[i].clone());
    }

    leftFeatureSet = _leftFeatureSet;
    stereo = imgs.size() == 2 ? true : false;
    IdGenerator *idGenerator = IdGenerator::createInstance();
    id = idGenerator->next();
    time = _time;
    keyView = false;
}

View::View(View* v, int _time)
{
    imgs = v->getImgs();
    leftFeatureSet = v->getLeftFeatureSet();
    rightFeatureSet = v->getRightFeatureSet();
    pose = v->getPose().clone();
    stereo = imgs.size() == 2 ? true : false;
    IdGenerator *idGenerator = IdGenerator::createInstance();
    id = idGenerator->next();
    time = _time;
    keyView = false;
}

View::~View()
{
    cout << "destructor!!!!" << endl;
}
long View::getId()
{
    return id;
}
vector<Mat> View::getImgs()
{
    vector<Mat> _imgs;
    for(int i = 0; i < imgs.size(); i++)
    {
        _imgs.push_back(imgs[i].clone());
    }
    return _imgs;
}
void View::setImgs(const vector<Mat> _imgs)
{
    imgs = vector<Mat>();
    for(int i = 0; i < _imgs.size(); i++)
    {
        imgs.push_back(_imgs[i].clone());
    }

}
void View::deleteImgs(
)
{
    for(int i = 0; i < imgs.size(); i++)
        imgs[i].release();
}
FeatureSet& View::getLeftFeatureSet()
{
    return leftFeatureSet;
}
FeatureSet& View::getRightFeatureSet()
{
    return rightFeatureSet;
}
void View::setLeftFeatures(const FeatureSet _leftFeatureSet)
{
    leftFeatureSet = _leftFeatureSet;
}
void View::setRightFeatures(const FeatureSet _rightFeatureSet)
{
    rightFeatureSet = _rightFeatureSet;
}
Mat View::getPose()
{
    return pose.clone();
}
void View::setPose(const Mat _pose)
{
    pose = _pose.clone();
}
void View::setPose(const Mat _R, const Mat _T)
{
    pose = Mat::eye(4, 4, CV_64F);
    Mat aux = pose(Rect(0, 0, 3, 3));
    _R.copyTo(aux);
    aux = pose(Rect(3, 0, 1, 3));
    _T.copyTo(aux);
}
Mat View::getR()
{
    return pose(Rect(0, 0, 3, 3)).clone();
}
Mat View::getT()
{
    return pose(Rect(3, 0, 1, 3)).clone();
}
