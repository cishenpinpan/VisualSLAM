//
//  SystemParameters.h
//  VisualSLAM
//
//  Created by Rong Yuan on 3/7/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#ifndef SystemParameters_h
#define SystemParameters_h

#define FEATURE_NAME "SURF"
#define DESCRIPTOR_NAME "SURF"
#define FEATURE_REDETECTION_TRIGGER 200

#define NUM_POSES 4660
#define BUNDLE_ADJUSTMENT_LENGTH 5
#define KEYFRAME_INTERVAL 5

#define REPROJECTION_THRESHOLD 5.0

#define MOTION_ONLY 1
#define STRUCTURE_ONLY 2
#define MOTION_STRUCTURE 3
#define GLOBAL_BA true
#define LOCAL_BA false

#define TRINOCULAR true
#define TRIANGULATION false
#define ONEDRANSAC true


#define RATIO_ERROR_THRESHOLD 0.08
#define SPEED_ERROR_THRESHOLD 0.15

#define RANSAC_CONFIDENCE 0.9

#define TRACK "02"
#define START_FRAME 0
#define IMAGE_DIR "/home/hongyi/Documents//KITTI/"
#define GROUNDTRUTH_DIR "/home/hongyi/Documents//KITTI/GroundTruth/poses/"
#define OUTPUT_DIR "/home/hongyi/Documents//KITTI/Results/"

#endif /* SystemParameters_h */
