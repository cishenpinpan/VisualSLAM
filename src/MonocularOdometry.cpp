//
//  MonocularOdometry.cpp
//  VisualSLAM
//
//  Created by Rong Yuan on 3/9/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#include "MonocularOdometry.h"



vector<int> keyFrames;

void MonocularOdometry::run(ViewReader *reader, FeatureExtractor *featureExtractor, FeatureTracker *featureTracker,    ViewTracker *viewTracker, PoseEstimator *poseEstimator, vector<View*> &views, bool trinocular)
{
    // if views are not given
    if(!views.size())
    {
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
        bool stereo = false;
        
        for(int i = 2; i <= NUM_POSES; i++)
        {
            cout << "frame: " << i << endl;
            // 1. read in next image
            images = reader->next();
            vector<Mat> currImgs = images;
            
            // 2. initialize next view
            View *currView = new View(currImgs, i);
            viewTracker->addView(currView);
            
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
    }
    
    // compute poses given feature correspondences
    vector<View*> allViews = viewTracker->getViews();
    vector<View*> keyViews = viewTracker->getKeyViews();
    // poseEstimator->solvePosesPnPStereo(keyViews);
    poseEstimator->refineScaleStereo(keyViews, trinocular);
    // poseEstimator->solveRatioInTriplets(keyViews, allViews);
   
    views = viewTracker->getKeyViews();
}


void MonocularOdometry::save(string track, vector<View*> views, string filename)
{
    // write poses to file
    ofstream output;
    output.open(OUTPUT_DIR + track + "_" + filename + ".output");
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
    output.open(OUTPUT_DIR + track + "_" + filename + "KeyFrames.output");
    for(int i = 0; i < views.size(); i++)
    {
        if(views[i]->isKeyView())
        {
            output << views[i]->getTime();
            output << "\n";
        }
        
    }
    output.close();
}

