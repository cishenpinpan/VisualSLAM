//
//  Frame.cpp
//  VisualSLAM
//
//  Created by Rong Yuan on 2/24/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#include "View.h"

View::View(const vector<Mat> _imgs)
{
    imgs = vector<Mat>();
    for(int i = 0; i < _imgs.size(); i++)
    {
        imgs.push_back(_imgs[i]);
    }
    idBook = map<long, int>();
    stereo = imgs.size() == 2 ? true : false;
    IdGenerator *idGenerator = IdGenerator::createInstance();
    id = idGenerator->next();
    keyView = false;
}

View::View(const vector<Mat> _imgs, const FeatureSet _leftFeatureSet)
{
    imgs = vector<Mat>();
    for(int i = 0; i < _imgs.size(); i++)
    {
        imgs.push_back(_imgs[i]);
    }

    leftFeatureSet = _leftFeatureSet;
    // initialize idBook
    vector<long> ids = leftFeatureSet.getIds();
    for(int i = 0; i < ids.size(); i++)
    {
        idBook.insert(make_pair(ids[i], i));
    }
    stereo = imgs.size() == 2 ? true : false;
    IdGenerator *idGenerator = IdGenerator::createInstance();
    id = idGenerator->next();
    keyView = false;
}

View::View(View* v)
{
    imgs = v->getImgs();
    leftFeatureSet = v->getLeftFeatureSet();
    rightFeatureSet = v->getRightFeatureSet();
    idBook = v->getIdBook();
    pose = v->getPose().clone();
    stereo = imgs.size() == 2 ? true : false;
    IdGenerator *idGenerator = IdGenerator::createInstance();
    id = idGenerator->next();
    keyView = false;
}

View::~View()
{
    deleteImgs();
    idBook.clear();
    pose.release();
}
long View::getId()
{
    return id;
}
vector<Mat> View::getImgs()
{
    vector<Mat> _imgs = vector<Mat>();
    for(int i = 0; i < imgs.size(); i++)
    {
        _imgs.push_back(imgs[i]);
    }
    return _imgs;
}
void View::setImgs(const vector<Mat> _imgs)
{
    imgs = vector<Mat>();
    for(int i = 0; i < _imgs.size(); i++)
    {
        imgs.push_back(_imgs[i]);
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

Feature View::getLeftFeatureById(long id)
{
    if(idBook.count(id) == 0)
    {
        cout << "Point id does not exist." << endl;
        return Feature(KeyPoint({0.0, 0.0}, 1.f), -1);
    }
    return leftFeatureSet.getFeatureByIndex(idBook[id]);
}
void View::removeLeftFeatureById(long id)
{
    leftFeatureSet.removeFeature(id);
    idBook.erase(id);
}
void View::removeRightFeatureById(long id)
{
    rightFeatureSet.removeFeature(id);
    idBook.erase(id);
}
Feature View::getRightFeatureById(long id)
{
    if(idBook.count(id) == 0)
    {
        cout << "Point id does not exist." << endl;
        return Feature(KeyPoint({0.0, 0.0}, 1.f), -1);
    }
    return rightFeatureSet.getFeatureByIndex(idBook[id]);
}

void View::setLeftFeatures(const FeatureSet _leftFeatureSet)
{
    leftFeatureSet = _leftFeatureSet;
}
void View::setRightFeatures(const FeatureSet _rightFeatureSet)
{
    rightFeatureSet = _rightFeatureSet;
}

map<long, int>& View::getIdBook()
{
    return idBook;
}

void View::setIdBook(const map<long, int> _idBook)
{
    idBook = _idBook;
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


Mat View::getRelativePose(View* v)
{
    Mat relativePose = pose.inv() * v->getPose();
    return relativePose.clone();
}

Mat View::getRelativeR(View *v)
{
    Mat relativeR = getRelativePose(v)(Rect(0, 0, 3, 3));
    return relativeR.clone();
}

Mat View::getRelativeT(View *v)
{
    Mat relativeT = getRelativePose(v)(Rect(3, 0, 1, 3));
    return relativeT.clone();
}

void View::setGroundTruth(const Mat _gt)
{
    gt = _gt.clone();
}
Mat View::getGroundTruth()
{
    return gt.clone();
}
