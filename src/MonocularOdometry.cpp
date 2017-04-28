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
            // poseEstimator->estimatePose(lastKeyView, currView);
            keyFrames.push_back(i);
        }
        
        // 3. update view ptrs
        prevView = currView;
    }
    
    // optimize triplets
    vector<View*> allViews = viewTracker->getViews();
    vector<View*> keyViews = viewTracker->getKeyViews();
    // poseEstimator->solvePosesPnPStereo(keyViews);
    // poseEstimator->refineScaleStereo(keyViews);
    poseEstimator->solveRatioInTriplets(keyViews, allViews);
   
    
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

