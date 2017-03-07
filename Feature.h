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
#include "opencv2/features2d.hpp"

using namespace std;
using namespace cv;

class Feature
{
private:
    Point2f point2d;
    long int id;
public:
    Feature();
    Feature(Point2f _point2d) : point2d(_point2d){id = -1;};
    Feature(long int _id) : id(_id){point2d = {0.0, 0.0};};
    Feature(Point2f _point2d, long _id) : point2d(_point2d), id(_id){};
    Point2f getPoint(){return point2d;}
    long getId(){return id;}
};

class FeatureSet
{
private:
    vector<Point2f> featurePoints;
    vector<long int> ids;
public:
    FeatureSet(){;};
    FeatureSet(vector<Point2f> _featurePoints, vector<long int> _ids) : featurePoints(_featurePoints), ids(_ids){};
    vector<Point2f>& getFeaturePoints(){return featurePoints;};
    void setFeaturePoints(const vector<Point2f> _featurePoints){featurePoints = vector<Point2f>(_featurePoints);};
    vector<long int>& getIds(){return ids;};
    void setIds(const vector<long int> _ids){ids = vector<long int>(_ids);};
    Feature getFeatureByIndex(int index){return Feature(featurePoints[index], ids[index]);}
    void addFeature(Feature feature);
    void addFeature(Point2f, const long id);
};

#endif /* Feature_h */
