//
//  MotionModel.hpp
//  VisualSLAM
//
//  Created by Rong Yuan on 4/25/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#ifndef MotionModel_h
#define MotionModel_h

#include <stdio.h>
#include "View.h"

using namespace std;

class MotionModel
{
public:
    double getDistanceRatio(View *v1, View *v2, View *v3)
    {
        return double((v3->getTime() - v2->getTime())) / double((v2->getTime() - v1->getTime()));
    }
    double getDistance(vector<View*> views, View *nextView)
    {
        double speed = getSpeed(views[views.size() - 2], views[views.size() - 1]);
        double dist = speed * (nextView->getTime() - views.back()->getTime());
        return dist;
    }
private:
    double getSpeed(View* v1, View *v2)
    {
        int t1 = v1->getTime();
        int t2 = v2->getTime();
        Mat relativePose = v1->getPose().inv() * v2->getPose();
        double dist = norm(relativePose.col(3).rowRange(0, 3));
        double speed = dist / (t2 - t1);
        return speed;
    }
    
    
};
#endif /* MotionModel_hpp */
