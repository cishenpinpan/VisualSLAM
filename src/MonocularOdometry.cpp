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
            viewTracker->bundleAdjust(MOTION_STRUCTURE, GLOBAL_BA);
            
            // local keyview tracker
            localViewTracker->addView(currView);
            localViewTracker->computeLandmarks(true);
            localViewTracker->bundleAdjust(MOTION_STRUCTURE, GLOBAL_BA);
        }
        else
        {
            
            featureTracker->kltTrack(prevView->getImgs()[0], currView->getImgs()[0],
                                     prevView->getLeftFeatureSet(), currView->getLeftFeatureSet(), false);
//
//            // 3. estimate pose from PnP (motion-only BA)
//            double inliers = poseEstimator->solvePnP(currView, viewTracker->getLandmarkBook());
//            // refine landmarks
//            // viewTracker->bundleAdjust(STRUCTURE_ONLY, LOCAL_BA);
//            if(inliers < 0.7)
//            {
//                cout << "inliers before BA: " << inliers * 100 << "%" << endl;
//                double inliers = poseEstimator->solvePnP(currView, viewTracker->getLandmarkBook());
//                cout << "inliers after BA: " << inliers * 100 << "%" << endl;
//            }
//            Tr = currView->getPose();
//            cout << Tr.col(3).rowRange(0, 3) << endl;
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
        if(currView->getLeftFeatureSet().size() < 0.9 * lastKeyView->getLeftFeatureSet().size() ||
            currView->getLeftFeatureSet().size() < FEATURE_REDETECTION_TRIGGER)
        {
            // 1. triplet
            //    - a) track down features in local key view tracker
            //    - b) PnP to solve for lambda (using local landmarks)
            //    - c) epipolar geometry to solve for orientation
            
            // a) track features
            View* prevKeyView = localViewTracker->getLastView();
            featureTracker->refineTrackedFeatures(prevKeyView->getImgs()[0], currView->getImgs()[0], prevKeyView->getLeftFeatureSet(), currView->getLeftFeatureSet(), false);
            // viewTracker->bundleAdjust(STRUCTURE_ONLY, LOCAL_BA);
            viewTracker->setKeyView(currView);
            localViewTracker->addView(currView);
            // b) PnP solves lambda of relative pose from last keyframe
            //    - a scale prior from classic PnP
            poseEstimator->solvePnP(currView, localViewTracker->getLandmarkBook());
            Mat relativePose = prevKeyView->getPose().inv() * currView->getPose();
            double scalePrior = norm(relativePose.col(3).rowRange(0, 3));
            //    - re-extract features
            featureExtractor->reextractFeatures(prevKeyView->getImgs()[0], prevKeyView->getLeftFeatureSet());
            int lastKeyFrame = keyFrames.back() - 1;
            vector<View*>::iterator start = viewTracker->getViews().begin() + lastKeyFrame,
            end = viewTracker->getViews().end();
            vector<View*> viewsFromLastKeyView(start, end);
            featureTracker->trackAndMatch(viewsFromLastKeyView);
            //    - and estimate essential matrix
            poseEstimator->estimatePose(prevKeyView, currView, scalePrior);
            //    - optimize scale within three frames
            poseEstimator->solveScalePnP(currView, prevKeyView->getPose(), localViewTracker->getLandmarkBook());

            // 2. quadruple
            vector<View*> fourKeyViews = viewTracker->getLastNKeyViews(4);
            if(!fourKeyViews.empty())
            {
                // refine scale factor
                poseEstimator->refineScale(fourKeyViews, viewTracker->getLandmarkBook());
                // landmarks from k2 and k3 need re-computation now
                viewTracker->refineLandmarks(fourKeyViews[1], fourKeyViews[2]);
            }
            cout << currView->getT() << endl;
            // 5. re-generate new landmarks
            viewTracker->computeLandmarks(false);
            keyFrames.push_back(i);
            
            // 6. update local keyView tracker
            localViewTracker->eraseLandmarks();
            localViewTracker->popFirstView();
            localViewTracker->computeLandmarks(true);
            
            vector<Point2f> ps1, ps2;
            ps1 = Converter::keyPointsToPoint2fs(prevKeyView->getLeftFeatureSet().getFeaturePoints());
            ps2 = Converter::keyPointsToPoint2fs(currView->getLeftFeatureSet().getFeaturePoints());
            Canvas canvas;
            // canvas.drawFeatureMatches(prevKeyView->getImgs()[0], currView->getImgs()[0], ps1, ps2);
            // refine landmarks
            // viewTracker->bundleAdjust(MOTION_STRUCTURE, GLOBAL_BA);
            // cout << currView->getT() << endl;
            
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

