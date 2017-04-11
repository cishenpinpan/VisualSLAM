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
    input.open("/Users/orangechicken/Documents/MATLAB/KITTI/GroundTruth/poses/" + track + ".txt");
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
            featureTracker->track(prevView->getImgs()[0], currView->getImgs()[0], prevView->getLeftFeatureSet(), currView->getLeftFeatureSet(), false);
            
            Mat poseChange = poseEstimator->estimatePose(prevView, currView);
            poseChange.at<double>(0, 3) = poseChange.at<double>(0, 3) * 6.5;
            poseChange.at<double>(1, 3) = poseChange.at<double>(1, 3) * 6.5;
            poseChange.at<double>(2, 3) = poseChange.at<double>(2, 3) * 6.5;
            Tr = Tr * poseChange;
            currView->setPose(Tr);
            viewTracker->setKeyView(currView);
            keyFrames.push_back(i);
            viewTracker->bundleAdjust();
            Canvas canvas;
            vector<Point2f> ps1, ps2;
            KeyPoint::convert(prevView->getLeftFeatureSet().getFeaturePoints(), ps1);
            KeyPoint::convert(currView->getLeftFeatureSet().getFeaturePoints(), ps2);
//            canvas.drawFeatureMatches(prevView->getImgs()[0], currView->getImgs()[0], ps1, ps2);
        }
        else
        {
            featureTracker->searchLandmarks(currView, viewTracker->getLandmarkBook(), viewTracker->getLastKeyView());
            map<long, Landmark> landmarkBook = viewTracker->getLandmarkBook();
            
            cout << "features: " << currView->getLeftFeatureSet().size() << endl;
            // 3. estimate pose from PnP (motion-only BA)
            Tr = poseEstimator->solvePnP(currView, landmarkBook);
            cout << Tr.col(3).rowRange(0, 3) << endl;
            currView->setPose(Tr);
        }
        
        // 4. update view
        prevView = currView;
        // re-detect if necessary
        if(i != 0 && i - keyFrames.back() >= KEYFRAME_INTERVAL)
        {
            View* prevKeyView = viewTracker->getLastKeyView();
            featureExtractor->reextractFeatures(prevKeyView->getImgs()[0], prevKeyView->getLeftFeatureSet());
            featureTracker->track(prevKeyView->getImgs()[0], prevView->getImgs()[0], prevKeyView->getLeftFeatureSet(), prevView->getLeftFeatureSet(), false);
            viewTracker->setKeyView(prevView);
            keyFrames.push_back(i);
        }
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
    output.open("/Users/orangechicken/Desktop/SLAM/KITTI_" + track + "_MonoLeft.output");
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
    output.open("/Users/orangechicken/Desktop/SLAM/KITTI_" + track + "_KeyFrames.output");
    for(int i = 0; i < keyFrames.size(); i++)
    {
        output << keyFrames[i];
        output << "\n";
    }
    output.close();
}
