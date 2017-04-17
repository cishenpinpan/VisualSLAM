//
//  SystemParameters.h
//  VisualSLAM
//
//  Created by Rong Yuan on 3/7/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

//
//  SystemParameters.h
//  VisualSLAM
//
//  Created by Rong Yuan on 3/7/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#ifndef SystemParameters_h
#define SystemParameters_h

#define FEATURE_NAME "ORB"
#define DESCRIPTOR_NAME "ORB"
#define FEATURE_REDETECTION_TRIGGER 100

#define NUM_POSES 500
#define BUNDLE_ADJUSTMENT_LENGTH 5
#define KEYFRAME_INTERVAL 5

#define REPROJECTION_THRESHOLD 20

#define MOTION_ONLY 1
#define STRUCTURE_ONLY 2
#define MOTION_STRUCTURE 3
#define GLOBAL_BA true
#define LOCAL_BA false

#define TRACK "00"
#define START_FRAME 0
#define IMAGE_DIR "/Users/orangechicken/Documents/MATLAB/KITTI/"
#define GROUNDTRUTH_DIR "/Users/orangechicken/Documents/MATLAB/KITTI/GroundTruth/poses/"
#define OUTPUT_DIR "/Users/orangechicken/Desktop/SLAM/KITTI_"

#endif /* SystemParameters_h */
