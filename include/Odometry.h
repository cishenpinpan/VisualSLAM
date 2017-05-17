//
//  MonocularOdometry.hpp
//  VisualSLAM
//
//  Created by Rong Yuan on 3/9/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#ifndef MonocularOdometry_h
#define MonocularOdometry_h

#include <fstream>
#include "ViewReader.h"
#include "ViewTracker.h"
#include "PoseEstimator.h"

namespace blindfind
{
    class View;
    class Odometry
    {
    public:
        void run(vector<View*> &views, bool trinocular);
        void save(string _track, vector<View*> _views, string filename);
    private:
        vector<Mat> readGroundTruth(string track);
    };
}
#endif /* MonocularOdometry_h */
