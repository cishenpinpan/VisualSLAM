//
//  MotionModel.hpp
//  VisualSLAM
//
//  Created by Rong Yuan on 4/25/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#ifndef MotionModel_h
#define MotionModel_h

using namespace std;

class MotionModel
{
public:
    double getSpeed(double s1, double t1, double s2, double t2, double t)
    {
        double a = (s2 -s1) / (t2 - t1);
        double s = s2 + a * (t - t2);
        return s;
    }
};
#endif /* MotionModel_hpp */
