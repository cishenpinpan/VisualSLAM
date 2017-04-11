//
//  Feature.cpp
//  VisualSLAM
//
//  Created by Rong Yuan on 3/7/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#include "Feature.h"

void FeatureSet::clear()
{
    // clear feature points and ids
    featurePoints.clear();
    ids.clear();
}
void FeatureSet::addFeature(Feature feature)
{
    addFeature(feature.getPoint(), feature.getId());
}
Feature FeatureSet::getFeatureById(long id)
{
    for(int i = 0; i < ids.size(); i++)
    {
        if(ids[i] == id)
            return Feature(featurePoints[i], id);
    }
    return NULL;
}
void FeatureSet::addFeature(KeyPoint point2d, long id)
{
    featurePoints.push_back(point2d);
    ids.push_back(id);
}
void FeatureSet::removeFeature(const long id)
{
    vector<KeyPoint> newPoints;
    vector<long> newIds;
    for(int i = 0; i < featurePoints.size(); i++)
    {
        if(ids[i] != id)
        {
            newPoints.push_back(featurePoints[i]);
            newIds.push_back(ids[i]);
        }
    }
    setFeaturePoints(newPoints);
    setIds(newIds);
}
int FeatureSet::size()
{
    return featurePoints.size();
}
bool FeatureSet::hasId(long id)
{
    for(int i = 0; i < ids.size(); i++)
    {
        if(ids[i] == id)
            return true;
    }
    return false;
}
