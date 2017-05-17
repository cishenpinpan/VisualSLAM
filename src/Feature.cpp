//
//  Feature.cpp
//  VisualSLAM
//
//  Created by Rong Yuan on 3/7/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#include "Feature.h"

using namespace blindfind;

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
Feature FeatureSet::getFeatureById(long id) const
{
    if(!idLookup.count(id))
        return NULL;
    int index = idLookup.at(id);
    return Feature(featurePoints[index], ids[index]);
    
}
void FeatureSet::addFeature(KeyPoint point2d, long id)
{
    featurePoints.push_back(point2d);
    ids.push_back(id);
    idLookup[id] = int(ids.size() - 1);
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
    return int(featurePoints.size());
}
bool FeatureSet::hasId(long id)
{
    return idLookup.count(id);
}
void FeatureSet::setIds(const vector<long> _ids)
{
    ids = vector<long int>(_ids);
    // update id lookup
    idLookup.clear();
    for(int i = 0; i < ids.size(); i++)
    {
        idLookup[ids[i]] = i;
    }
}
