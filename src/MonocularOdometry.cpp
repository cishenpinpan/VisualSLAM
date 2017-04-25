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
    
    View *prevView = new View(images, 1);
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
    
    // build a local key view tracker
    ViewTracker *localViewTracker = new ViewTracker();
    localViewTracker->addView(prevView);
    
    for(int i = 2; i < NUM_POSES; i++)
    {
        if((i + 1) % 1 == 0)
            cout << "frame: " << i << endl;
        // 1. read in next image
        images = reader->next();
        vector<Mat> currImgs = images;
        
        // 2. initialize next view
        View *currView = new View(currImgs, i);
        viewTracker->addView(currView);
        currView->setPose(Tr);
        
        View *lastKeyView = viewTracker->getLastKeyView();
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
            featureTracker->trackAndMatch(currViews);
            
            Mat poseChange = poseEstimator->estimatePose(prevView, currView);
            poseChange.at<double>(0, 3) = poseChange.at<double>(0, 3) * 2.7;
            poseChange.at<double>(1, 3) = poseChange.at<double>(1, 3) * 2.7;
            poseChange.at<double>(2, 3) = poseChange.at<double>(2, 3) * 2.7;
            Tr = Tr * poseChange;
            cout << Tr.col(3).rowRange(0, 3) << endl;
            currView->setPose(Tr);
            viewTracker->setKeyView(currView);
            viewTracker->computeLandmarks(true);
            keyFrames.push_back(i);
            // viewTracker->bundleAdjust(MOTION_STRUCTURE, GLOBAL_BA);
            
            // local keyview tracker
            localViewTracker->addView(currView);
            localViewTracker->computeLandmarks(true);
            // localViewTracker->updateLandmarks(viewTracker->getLandmarkBook());
        }
        else
        {
            
            featureTracker->kltTrack(prevView->getImgs()[0], currView->getImgs()[0],
                                     prevView->getLeftFeatureSet(), currView->getLeftFeatureSet(), false);
            
            featureTracker->refineTrackedFeatures(lastKeyView->getImgs()[0], currView->getImgs()[0], lastKeyView->getLeftFeatureSet(), currView->getLeftFeatureSet(), false);
            // poseEstimator->solvePnP(currView, localViewTracker->getLandmarkBook());
            
            
            cout << "features:" << currView->getLeftFeatureSet().size() << endl;
            
            
            // insert new keyframe when
            // 1. #KEYFRAME_INTERVAL frames has passed
            // 2. #features has dropped below the threshold
            if(currView->getLeftFeatureSet().size() < 0.5 * lastKeyView->getLeftFeatureSet().size() ||
               currView->getLeftFeatureSet().size() < FEATURE_REDETECTION_TRIGGER)
            {
                // 1. triplet
                //    - a) track down features in local key view tracker
                //    - b) epipolar geometry to solve for orientation
                //    - c) PnP to solve for lambda (using local landmarks)
                
                
                // a) track features
                // (tracked)
                viewTracker->bundleAdjust(STRUCTURE_ONLY, LOCAL_BA);
                localViewTracker->updateLandmarks(viewTracker->getLandmarkBook());
                viewTracker->setKeyView(currView);
                localViewTracker->addView(currView);
                // b) PnP solves lambda of relative pose from last keyframe
                //    - a scale prior from classic PnP
                //    Essential matrix solves for an orientation
                Mat relativePose = lastKeyView->getPose().inv() * currView->getPose();
                double scalePrior = norm(relativePose.col(3).rowRange(0, 3));
                cout << "prior: " << scalePrior << endl;
                //    - re-extract features
                featureExtractor->reextractFeatures(lastKeyView->getImgs()[0], lastKeyView->getLeftFeatureSet());
                int lastKeyFrame = keyFrames.back() - 1;
                vector<View*>::iterator start = viewTracker->getViews().begin() + lastKeyFrame,
                end = viewTracker->getViews().end();
                vector<View*> viewsFromLastKeyView;
                for(int i = lastKeyFrame; i < viewTracker->getViews().size(); i++)
                    viewsFromLastKeyView.push_back(viewTracker->getViews()[i]);
                featureTracker->trackAndMatch(viewsFromLastKeyView);
                //    - and estimate essential matrix
                poseEstimator->estimatePose(lastKeyView, currView, scalePrior);
                // c) PnP to solve for lambda (using local landmarks)
                //    - optimize scale within three frames
                vector<View*> triplet = viewTracker->getLastNKeyViews(3);
                poseEstimator->solveScalePnP(currView, lastKeyView->getPose(), localViewTracker->getLandmarkBook());
                // 2. quadruple
                vector<View*> fourKeyViews = viewTracker->getLastNKeyViews(4);
                //            if(!fourKeyViews.empty())
                //            {
                //                // refine scale factor
                //                poseEstimator->refineScale(fourKeyViews, viewTracker->getLandmarkBook());
                //                // landmarks from k2 and k3 need re-computation now
                //                viewTracker->refineLandmarks(fourKeyViews[1], fourKeyViews[2]);
                //            }
                cout << lastKeyView->getT() << endl;
                cout << "-------" << endl;
                cout << currView->getT() << endl;
                // 5. re-generate new landmarks
                viewTracker->computeLandmarks(false);
                // viewTracker->bundleAdjust(STRUCTURE_ONLY, LOCAL_BA);
                keyFrames.push_back(i);
                
                // 6. update local keyView tracker
                localViewTracker->eraseLandmarks();
                localViewTracker->popFirstView();
                localViewTracker->computeLandmarks(true);
                // localViewTracker->bundleAdjust(STRUCTURE_ONLY, LOCAL_BA);
                
                // refine landmarks
                // viewTracker->bundleAdjust(MOTION_STRUCTURE, GLOBAL_BA);
                // cout << currView->getT() << endl;
                
                cout << "KEYFRAME: features increased to: " << currView->getLeftFeatureSet().size() <<  endl;
            }
            else
            {
                featureTracker->kltTrack(prevView->getImgs()[0], currView->getImgs()[0],
                                         prevView->getLeftFeatureSet(), currView->getLeftFeatureSet(), false);
            }
        }
        
        
        
        // 4. update view
        prevView = currView;
        // update Tr
        Tr = viewTracker->getLastView()->getPose();
        
        Trs.push_back(Tr.clone());
    }
    views = viewTracker->getViews();
    for(int i = 0; i < views.size(); i++)
    {
        cout << "frame(" << i + 1 << "): " << endl;
        cout << views[i]->getT() << endl;
    }
}

void MonocularOdometry::run2(ViewReader *reader, FeatureExtractor *featureExtractor, FeatureTracker *featureTracker,    ViewTracker *viewTracker, PoseEstimator *poseEstimator, vector<Mat> &Trs, vector<View*> &views)
{
    // read ground truth
    vector<vector<double>> gt = readGroundTruth(NUM_POSES, reader);
    // initial pose
    Mat Tr = Mat::eye(4, 4, CV_64F);
    
    // initial image
    vector<Mat> images = reader->next();
    
    View *prevView = new View(images, 1);
    prevView->setPose(Mat::eye(4, 4, CV_64F));
    prevView->setGroundTruth(Mat::eye(4, 4, CV_64F));
    // extract features for it
    featureExtractor->extractFeatures(prevView->getImgs()[0], prevView->getLeftFeatureSet());
    featureTracker->trackAndMatch(prevView);
    
    list<detail::CameraParams> cameraPoses;
    
    // build an initial view tracker
    viewTracker->addView(prevView);
    // set first view as key view
    viewTracker->setKeyView(prevView);
    keyFrames.push_back(1);
    cout << "frame: 1" << endl;
    
    // build a local key view tracker
    ViewTracker *localViewTracker = new ViewTracker();
    localViewTracker->addView(prevView);
    
    for(int i = 2; i <= NUM_POSES; i++)
    {
        cout << "frame: " << i << endl;
        // 1. read in next image
        images = reader->next();
        vector<Mat> currImgs = images;
        
        // 2. initialize next view
        View *currView = new View(currImgs, i);
        viewTracker->addView(currView);
        currView->setPose(Tr);
        
        View *lastKeyView = viewTracker->getLastKeyView();
        
        featureTracker->kltTrack(prevView->getImgs()[0], currView->getImgs()[0],
                                 prevView->getLeftFeatureSet(), currView->getLeftFeatureSet(), false);
        if(currView->getLeftFeatureSet().size() < 0.8 * lastKeyView->getLeftFeatureSet().size() ||
           currView->getLeftFeatureSet().size() < FEATURE_REDETECTION_TRIGGER)
        {
            featureTracker->refineTrackedFeatures(lastKeyView->getImgs()[0], currView->getImgs()[0], lastKeyView->getLeftFeatureSet(), currView->getLeftFeatureSet(), false);
            cout << lastKeyView->getLeftFeatureSet().size() << "->" << currView->getLeftFeatureSet().size() << endl;
            viewTracker->setKeyView(currView);
            featureTracker->trackAndMatch(currView);
            featureExtractor->reextractFeatures(lastKeyView->getImgs()[0], lastKeyView->getLeftFeatureSet());
            int lastKeyFrame = keyFrames.back() - 1;
            vector<View*> viewsFromLastKeyView;
            for(int i = lastKeyFrame; i < viewTracker->getViews().size(); i++)
                viewsFromLastKeyView.push_back(viewTracker->getViews()[i]);
            featureTracker->trackAndMatch(viewsFromLastKeyView);
            poseEstimator->estimatePose(lastKeyView, currView);
            keyFrames.push_back(i);
        }
        
        // 3. update view ptrs
        prevView = currView;
    }
    
    // optimize triplets
    vector<View*> keyViews = viewTracker->getKeyViews();
    // poseEstimator->refineScaleStereo(keyViews);
    poseEstimator->refineScaleMultipleFrames(keyViews, 3);
   
    // optimize quadruples
    // poseEstimator->refineScaleMultipleFrames(keyViews, 4);
    
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

