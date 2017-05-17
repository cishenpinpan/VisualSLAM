//
//  MonocularOdometry.cpp
//  VisualSLAM
//
//  Created by Rong Yuan on 3/9/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#include "Odometry.h"

using namespace blindfind;

vector<int> keyFrames;

vector<Mat> Odometry::readGroundTruth(string track)
{
    vector<Mat> poses;
    string fileDir = IMAGE_DIR;
    fileDir = fileDir + "GroundTruth/poses/" + track + ".txt";
    ifstream in(fileDir);
    double num = 0.0;
    vector<double> nums;
    while(in >> num)
    {
        nums.push_back(num);
    }
    int i = 0;
    while(i < nums.size())
    {
        Mat pose = Mat::eye(4, 4, CV_64F);
        for(int j = 0; j < 12; j++, i++)
        {
            pose.at<double>(j / 4, j % 4) = nums[i];
        }
        poses.push_back(pose.clone());
    }
    vector<Mat>::iterator start = poses.begin() + START_FRAME, end = start + NUM_POSES;
    poses = vector<Mat>(start, end);
    return poses;
}

void Odometry::run(vector<View*> &views, bool trinocular)
{
    ViewReader *reader = new ViewReader("KITTI", TRACK, true);
    FeatureExtractor *featureExtractor = new FeatureExtractor();
    FeatureTracker *featureTracker = new FeatureTracker();
    ViewTracker *viewTracker = new ViewTracker();
    PoseEstimator *poseEstimator = new PoseEstimator();
    // vector<Mat> groundTruths = readGroundTruth(TRACK);
    
    // if views are not given
    if(!views.size())
    {
        // initial image
        vector<Mat> images = reader->next();
        
        View *prevView = new View(images, 1);
        prevView->setPose(Mat::eye(4, 4, CV_64F));
        // prevView->setGt(groundTruths[0].clone());
        // extract features for it
        featureExtractor->extractFeatures(prevView->getImgs()[0], prevView->getLeftFeatureSet());
        // featureExtractor->extractFeatures(prevView->getImgs()[1], prevView->getRightFeatureSet());
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
        bool stereo = true;
        
        for(int i = 2; i <= NUM_POSES; i++)
        {
            cout << "frame: " << i << endl;
            // 1. read in next image
            images = reader->next();
            vector<Mat> currImgs = images;
            
            // 2. initialize next view
            View *currView = new View(currImgs, i);
            // currView->setGt(groundTruths[i - 1].clone());
            viewTracker->addView(currView);
            
            View *lastKeyView = viewTracker->getLastKeyView();
            
            featureTracker->kltTrack(prevView->getImgs()[0], currView->getImgs()[0],
                                     prevView->getLeftFeatureSet(), currView->getLeftFeatureSet());
//            featureTracker->kltTrack(prevView->getImgs()[1], currView->getImgs()[1],
//                                     prevView->getRightFeatureSet(), currView->getRightFeatureSet());
            if(currView->getLeftFeatureSet().size() < 0.9 * lastKeyView->getLeftFeatureSet().size() ||
               currView->getLeftFeatureSet().size() < FEATURE_REDETECTION_TRIGGER || stereo)
            {
                featureTracker->refineTrackedFeatures(lastKeyView->getImgs()[0], currView->getImgs()[0], lastKeyView->getLeftFeatureSet(), currView->getLeftFeatureSet());
//                featureTracker->refineTrackedFeatures(lastKeyView->getImgs()[1], currView->getImgs()[1], lastKeyView->getRightFeatureSet(), currView->getRightFeatureSet());
                cout << lastKeyView->getLeftFeatureSet().size() << "->"<< currView->getLeftFeatureSet().size() << endl;
                //
                Canvas *canvas = new Canvas();
                vector<KeyPoint> kps1, kps2;
                for(int i = 0; i < currView->getLeftFeatureSet().size(); i++)
                {
                    long id = currView->getLeftFeatureSet().getIds()[i];
                    if(!lastKeyView->getLeftFeatureSet().hasId(id))
                        continue;
                    kps1.push_back(lastKeyView->getLeftFeatureSet().getFeatureById(id).getPoint());
                    kps2.push_back(currView->getLeftFeatureSet().getFeatureById(id).getPoint());
                }
                vector<Point2f> ps1, ps2;
                KeyPoint::convert(kps1, ps1);
                KeyPoint::convert(kps2, ps2);
                canvas->drawKeyPoints(lastKeyView->getImgs()[0], lastKeyView->getLeftFeatureSet().getFeaturePoints(), "what?");
                canvas->drawFeatureMatches(lastKeyView->getImgs()[0], currView->getImgs()[0], ps1, ps2);
                viewTracker->setKeyView(currView);
                featureTracker->trackAndMatch(currView);
                featureExtractor->reextractFeatures(currView->getImgs()[0], currView->getLeftFeatureSet());
//                featureExtractor->reextractFeatures(currView->getImgs()[1], currView->getRightFeatureSet());
                keyFrames.push_back(i);
            }
            
            // 3. update view ptrs
            prevView = currView;
        }
    }
    
    // compute poses given feature correspondences
    vector<View*> allViews = viewTracker->getViews();
    vector<View*> keyViews = viewTracker->getKeyViews();
    
    // PnP
    poseEstimator->solvePosesPnPStereo(keyViews);
    views = viewTracker->getKeyViews();
    save(TRACK, views, "P6P");

    
    // P1P
    poseEstimator->solveScale(allViews, keyViews, trinocular);
    views = viewTracker->getKeyViews();
    save(TRACK, views, "P3P");
    
    // Monocular
    // poseEstimator->solveRatioInTriplets(keyViews, allViews);
    
}


void Odometry::save(string track, vector<View*> views, string filename)
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
    output.open(OUTPUT_DIR + track + "_" + filename + ".keyFrames");
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

