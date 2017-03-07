//
//  Feature.cpp
//  VisualSLAM
//
//  Created by Rong Yuan on 3/7/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#include "Feature.h"

void FeatureSet::addFeature(Feature feature)
{
    addFeature(feature.getPoint(), feature.getId());
}
void FeatureSet::addFeature(Point2f point2d, long id)
{
    featurePoints.push_back(point2d);
    ids.push_back(id);
}
