//
//  SystemParameters.h
//  VisualSLAM
//
//  Created by Rong Yuan on 3/7/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#ifndef SystemParameters_h
#define SystemParameters_h

#define FEATURE_REDETECTION_TRIGGER 50
#define NUM_POSES 200
#define BUNDLE_ADJUSTMENT_LENGTH 10
#define KEYFRAME_INTERVAL 6

#define FEATURE_NAME "ORB"
#define DESCRIPTOR_NAME "ORB"

#define REPROJECTION_THRESHOLD 20

#define MOTION_ONLY 1
#define STRUCTURE_ONLY 2
#define MOTION_STRUCTURE 3
#define KEYVIEW_ONLY true
#define ALL_VIEWS false

#define TRACK "00"
#define IMAGE_DIR "/home/hongyi/Documents/KITTI/"
#define GROUNDTRUTH_DIR "/home/hongyi/Documents/KITTI/GroundTruth/poses/"
#define OUTPUT_DIR "/home/hongyi/Documents/KITTI/results/"

#endif /* SystemParameters_h */
