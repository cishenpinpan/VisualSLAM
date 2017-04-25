//
//  Feature.hpp
//  VisualSLAM
//
//  Created by Rong Yuan on 3/7/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#ifndef Feature_h
#define Feature_h

#include <stdio.h>
#include <map>
#include "opencv2/features2d.hpp"

using namespace std;
using namespace cv;

class Feature
{
private:
    KeyPoint point2d;
    long int id;
public:
    Feature();
    Feature(KeyPoint _point2d) : point2d(_point2d){id = -1;};
    Feature(long int _id) : id(_id){point2d = KeyPoint({0.0, 0.0}, 1.f);};
    Feature(KeyPoint _point2d, long _id) : point2d(_point2d), id(_id){};
    KeyPoint getPoint(){return point2d;}
    long getId(){return id;}
};

class FeatureSet
{
private:
    vector<KeyPoint> featurePoints;
    vector<long int> ids;
    map<long, int> idLookup;
public:
    FeatureSet(){;};
    FeatureSet(vector<KeyPoint> _featurePoints, vector<long int> _ids) : featurePoints(_featurePoints), ids(_ids){};
    vector<KeyPoint>& getFeaturePoints(){return featurePoints;};
    void setFeaturePoints(const vector<KeyPoint> _featurePoints){featurePoints = vector<KeyPoint>(_featurePoints);};
    vector<long int>& getIds(){return ids;};
    void setIds(const vector<long int> _ids);
    Feature getFeatureByIndex(int index){return Feature(featurePoints[index], ids[index]);}
    Feature getFeatureById(long id);
    void addFeature(Feature feature);
    void addFeature(KeyPoint, const long id);
    void removeFeature(const long id);
    void clear();
    int size();
    bool hasId(long id);
};

class Landmark
{
public:
    Landmark(const Landmark &l){point3d = Point3d(l.point3d); id = l.id; from = l.from; descriptor = l.descriptor.clone(); inlier = l.inlier;}
    Landmark(Point3d _point3d, long _id, pair<long, long> _from) : point3d(_point3d), id(_id), from(_from){};
    Landmark(){point3d = Point3d(0, 0, 0); id = -1; from = {-1, -1};}
    Point3d point3d;
    long id;
    pair<long, long> from;
    Mat descriptor;
    bool inlier = true;
    
    Point3d getPoint(){return point3d;}
    void setPoint(const Point3d &_point3d){point3d = Point3d(_point3d);}
    Mat getDescriptor(){return descriptor.clone();}
    void setDescriptor(const Mat _descriptor){descriptor = _descriptor.clone();}
    bool isInlier(){return inlier;}
    void setInlier(bool _inlier){inlier = _inlier;}
};

#endif /* Feature_h */
