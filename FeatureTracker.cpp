//
//  FeatureTracker.cpp
//  VisualSLAM
//
//  Created by Rong Yuan on 2/28/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#include "FeatureTracker.h"

bool LEFT_CAMERA = true, RIGHT_CAMERA = false;

// general
Mat FeatureTracker::track(Mat img1, Mat img2,
                                    FeatureSet& featureSet1, FeatureSet& featureSet2, bool stereo)
{
    map<int, int> indexBook;
    //this function automatically gets rid of points for which tracking fails
    
    vector<float> err;
    Mat status;
    Size winSize=Size(21,21);
    TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
    
    calcOpticalFlowPyrLK(img1, img2, featureSet1.getFeaturePoints(), featureSet2.getFeaturePoints(),
                         status, err, winSize, 3, termcrit, 0, 0.001);
    
    // sync feature id in feature set 2
    featureSet2.setIds(featureSet1.getIds());
    
    // set false to those out-of-image features
    vector<Point2f> points2 = featureSet2.getFeaturePoints();;
    for(int i = 0; i < points2.size(); i++)
    {
        if(points2[i].x < 0 || points2[i].x > 1281 || points2[i].y < 0 || points2[i].y > 376)
        {
            status.at<bool>(i, 0) = 0;
        }
    }
    
    /**
        Notice that incorrect feature tracking results are NOT removed at this stage!!!
        Perform further operations out of the function according to the status returned
     **/
    return status.clone();
}

// stereo track
void FeatureTracker::track(View *v)
{
    ;
}
