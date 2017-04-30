//
//  MonocularOdometry.hpp
//  VisualSLAM
//
//  Created by Rong Yuan on 3/9/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#ifndef MonocularOdometry_h
#define MonocularOdometry_h

#include <stdio.h>
#include <fstream>

#include "ViewReader.h"
#include "CameraParameters.h"
#include "View.h"
#include "ViewTracker.h"
#include "PoseEstimator.h"
#include "SystemParameters.h"

#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/core/affine.hpp"
#include "opencv2/calib3d/calib3d_c.h"
#include "opencv2/plot.hpp"

using namespace std;
using namespace cv;

class MonocularOdometry
{
public:
    void run(ViewReader *_reader, FeatureExtractor *_extractor, FeatureTracker *_featureTracker,
             ViewTracker *_viewTracker, PoseEstimator *_poseEstimator, vector<View*> &track, bool trinocular, bool oneDRansac);
    void save(string _track, vector<View*> _views, string filename);
private:
    vector<vector<double>> readGroundTruth(int numPoses, ViewReader* _reader);
};

#endif /* MonocularOdometry_h */
