//
//  MonocularOdometry.cpp
//  VisualSLAM
//
//  Created by Rong Yuan on 3/9/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#include "MonocularOdometry.h"


vector<vector<double>> MonocularOdometry::readGroundTruth(int numPoses, ViewReader* reader)
{
    vector<vector<double>> gt;
    ifstream input;
    string track = reader->getTrack();
    input.open(GROUNDTRUTH_DIR + track + ".txt");
    for(int i = 0; i <= numPoses; i++)
    {
        vector<double> row(12, 0.0);
        for(int j = 0; j < 12; j++)
        {
            input >> row[j];
        }
        gt.push_back(row);
    }
    input.close();
    return gt;
}

vector<int> keyFrames;
void MonocularOdometry::run(ViewReader *reader, FeatureExtractor *featureExtractor, FeatureTracker *featureTracker,    ViewTracker *viewTracker, PoseEstimator *poseEstimator, vector<Mat> &Trs, vector<View*> &views)
{
    // read ground truth
    vector<vector<double>> gt = readGroundTruth(NUM_POSES, reader);
    
    // initial pose
    Mat Tr = Mat::eye(4, 4, CV_64F);
    
    // initial image
    vector<Mat> images = reader->next();
    
    View *prevView = new View(images);
    prevView->setPose(Mat::eye(4, 4, CV_64F));
    prevView->setGroundTruth(Mat::eye(4, 4, CV_64F));
    // extract features for it
    featureExtractor->extractFeatures(prevView->getImgs()[0], prevView->getLeftFeatureSet());
    
    
    list<detail::CameraParams> cameraPoses;
    
    // build an initial view tracker
    viewTracker->addView(prevView);
    // set first view as key view
    viewTracker->setKeyView(prevView);
    keyFrames.push_back(0);
    
    Trs.push_back(Tr.clone());
    
    for(int i = 2; i < NUM_POSES; i++)
    {
        if((i + 1) % 1 == 0)
            cout << "frame: " << i << endl;
        // 1. read in next image
        images = reader->next();
        vector<Mat> currImgs = images;
        
        // 2. initialize next view
        View *currView = new View(currImgs);
        viewTracker->addView(currView);
        currView->setPose(Tr);
        
        // skip first 5 frames
        if(i < KEYFRAME_INTERVAL)
        {
            continue;
        }
        // build initial map
        if(i == KEYFRAME_INTERVAL)
        {
            // track features
            vector<View*> currViews = viewTracker->getViews();
            for(int j = 0; j < i - 1; j++)
            {
                View* v1 = currViews[j], *v2 = currViews[j + 1];
                featureTracker->kltTrack(v1->getImgs()[0], v2->getImgs()[0], v1->getLeftFeatureSet(), v2->getLeftFeatureSet(), false);
            }
            // refine (Lowe's)
            View* v1 = currViews[0], *v2 = currViews.back();
            featureTracker->refineTrackedFeatures(v1->getImgs()[0], v2->getImgs()[0], v1->getLeftFeatureSet(), v2->getLeftFeatureSet(), v2->getLeftFeatureSet().getIds(), false);
            
            Mat poseChange = poseEstimator->estimatePose(prevView, currView);
            poseChange.at<double>(0, 3) = poseChange.at<double>(0, 3) * 3.0;
            poseChange.at<double>(1, 3) = poseChange.at<double>(1, 3) * 3.0;
            poseChange.at<double>(2, 3) = poseChange.at<double>(2, 3) * 3.0;
            Tr = Tr * poseChange;
            cout << Tr.col(3).rowRange(0, 3) << endl;
            currView->setPose(Tr);
            viewTracker->setKeyView(currView);
            viewTracker->computeLandmarks(true);
            keyFrames.push_back(i);
            viewTracker->bundleAdjust(MOTION_STRUCTURE, GLOBAL_BA);
        }
        else
        {
            
            featureTracker->kltTrack(prevView->getImgs()[0], currView->getImgs()[0],
                                     prevView->getLeftFeatureSet(), currView->getLeftFeatureSet(), false);
//            featureTracker->refineTrackedFeatures(prevView->getImgs()[0], currView->getImgs()[0],
//                                                  prevView->getLeftFeatureSet(), currView->getLeftFeatureSet(),
//                                                  currView->getLeftFeatureSet().getIds(), false);
            
            // featureTracker->searchLandmarks(currView, viewTracker->getLandmarkBook(), viewTracker->getLastKeyView());
            map<long, Landmark> landmarkBook = viewTracker->getLandmarkBook();
            
            // 3. estimate pose from PnP (motion-only BA)
            poseEstimator->solvePnP(currView, landmarkBook);
            
            // refine landmarks
            viewTracker->bundleAdjust(STRUCTURE_ONLY, LOCAL_BA);
            
            Tr = currView->getPose();
            cout << Tr.col(3).rowRange(0, 3) << endl;
        }
        cout << "features:" << currView->getLeftFeatureSet().size() << endl;
        
        // decline large motions
        View* lastKeyView = viewTracker->getLastKeyView();
        Mat relativePose = lastKeyView->getPose().inv() * currView->getPose();
        double dist = relativePose.at<double>(0, 3) * relativePose.at<double>(0, 3)
                    + relativePose.at<double>(1, 3) * relativePose.at<double>(1, 3)
                    + relativePose.at<double>(2, 3) * relativePose.at<double>(2, 3);
        dist = sqrt(dist);
        
        // insert new keyframe when
        // 1. #KEYFRAME_INTERVAL frames has passed
        // 2. #features has dropped below the threshold
        if(((i != 0 && i - keyFrames.back() >= KEYFRAME_INTERVAL) &&
            currView->getLeftFeatureSet().size() < 0.5 * lastKeyView->getLeftFeatureSet().size()) ||
            currView->getLeftFeatureSet().size() < 50)
        {
            viewTracker->setKeyView(currView);
            // refine the frame before nominating it as a keyframe
//            viewTracker->bundleAdjust(MOTION_STRUCTURE, GLOBAL_BA);
//            cout << viewTracker->getLastView()->getT() << endl;
            
            bool stereo = true;
            View* prevKeyView = viewTracker->getLastTwoKeyViews().front(),
                    currKeyView = viewTracker->getLastTwoKeyViews().back();
            featureExtractor->reextractFeatures(prevKeyView->getImgs()[0], prevKeyView->getLeftFeatureSet());
            
            vector<View*> currViews = viewTracker->getViews();
            for(int j = keyFrames.back() - 1; j < currViews.size() - 1; j++)
            {
                View* v1 = currViews[j], *v2 = currViews[j + 1];
                featureTracker->kltTrack(v1->getImgs()[0], v2->getImgs()[0], v1->getLeftFeatureSet(), v2->getLeftFeatureSet(), false);
            }
            // refine (Lowe's)
            View* v1 = currViews[keyFrames.back() - 1], *v2 = currViews.back();
            featureTracker->refineTrackedFeatures(v1->getImgs()[0], v2->getImgs()[0], v1->getLeftFeatureSet(), v2->getLeftFeatureSet(), v2->getLeftFeatureSet().getIds(), false);
            viewTracker->computeLandmarks(false);
            keyFrames.push_back(i);
            cout << "KEYFRAME: features increased to: " << currView->getLeftFeatureSet().size() <<  endl;
        }
        // 4. update view
        prevView = currView;
        // update Tr
        Tr = viewTracker->getLastView()->getPose();
        
        Trs.push_back(Tr.clone());
    }
    views = viewTracker->getViews();
}


void MonocularOdometry::save(string track, vector<View*> views)
{
    // write poses to file
    ofstream output;
    output.open(OUTPUT_DIR + track + "_MonoLeft.output");
    for(int i = 0; i < views.size(); i++)
    {
        Mat pose = views[i]->getPose();
        for(int row = 0; row < 4; row++)
        {
            for(int col = 0; col < 4; col++)
            {
                if(col != 0 || row != 0) // first
                    output << " ";
                output << double(pose.at<double>(row, col));
                if(col == 3 && row == 3 && i + 1 != views.size())
                    output << "\n";
                
            }
        }
    }
    output.close();
    output.open(OUTPUT_DIR + track + "_KeyFrames.output");
    for(int i = 0; i < keyFrames.size(); i++)
    {
        output << keyFrames[i];
        output << "\n";
    }
    output.close();
}

