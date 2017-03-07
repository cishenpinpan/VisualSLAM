//
//  FeatureTracker.cpp
//  VisualSLAM
//
//  Created by Rong Yuan on 2/24/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#include "ViewTracker.h"

ViewTracker::ViewTracker()
{
    featureTracker = new FeatureTracker();
    featureExtractor = new FeatureExtractor();
}
ViewTracker::~ViewTracker()
{
    ;
}
void ViewTracker::addView(View *v)
{
    views.push_back(v);
}

vector<View*> ViewTracker::getViews()
{
    return views;
}

View* ViewTracker::getPrevView()
{
    return views[views.size() - 2];
}

View* ViewTracker::getCurrView()
{
    return views.back();
}

View* ViewTracker::popLastView()
{
    View* temp = views.back();
    views.erase(views.end() - 1);
    return temp;
}



void ViewTracker::bundleAdjust()
{
    Sba sba;
//    int nFrames = views.size();
//    // 0. only bundle adjust if more than 3 views
//    if(views.size() < 3)
//        return ;
//    // 1. triangulate to get 3d points out of all feature tracks
//    // 1.1 take common features out of views under track (find from last view)
//    set<int> survivingFids = findSurvivingFids();
//    // each outer element corresponds to a view
//    // each inner array corresponds to feature points of certain view
//    vector<vector<Point2d>> leftFeatureTrack(nFrames, vector<Point2d>());
//    vector<vector<Point2d>> rightFeatureTrack(nFrames, vector<Point2d>());
//    for(set<int>::iterator idIter = survivingFids.begin(); idIter != survivingFids.end(); idIter++)
//    {
//        int id = *idIter;
//        for(int i = 0; i < nFrames; i++)
//        {
//            Point2d pt = Point2d(views[i]->getLeftFeatureById(id));
//            leftFeatureTrack[i].push_back(Point2d(pt.x, pt.y));
//            pt = Point2d(views[i]->getRightFeatureById(id));
//            rightFeatureTrack[i].push_back(Point2d(pt.x, pt.y));
//        }
//    }
//    // 1.2 triangulate !!
//    // (now only use first stereo view to construct initial 3d points
//    vector<Point3d> points3D;
//    
//    Mat relativePose = Mat::eye(4, 4, CV_64F);
//    relativePose.at<double>(0, 3) = 0.537;
//    triangulatePoints(views[0]->getPose(), relativePose,
//                      leftFeatureTrack[0], rightFeatureTrack[0], points3D);
//    // 2. build visibility map Visibility
//    //    of size (#Views x #Features)
//    vector<vector<int>> visibility(nFrames * 2, vector<int>(survivingFids.size(), 1));
//    // 3. camera matrices (intrinsic)
//    //    of size (#Views)
//    Mat K = CameraParameters::getIntrinsic();
////    K.at<double>(1, 1) = - K.at<double>(1, 1);
////    K.at<double>(1, 2) = 376 - K.at<double>(1, 2);
//    vector<Mat> cameraMatrices(nFrames * 2, K);
//    // 4. Rotation/Translation matrices
//    //    of size (#Views)
//    vector<Mat> rotationMatrices, translationMatrices, tCopy;
//    // 5. distortion coefficients (rectified -> all zeros)
//    vector<Mat> distCoefficients(nFrames * 2, Mat::zeros(5, 1, CV_64F));
//    
//    // left cameras
//    for(int i = 0; i < nFrames; i++)
//    {
//        Mat pR;
//        transpose(views[i]->getR(), pR);
//        Mat pT = -pR * views[i]->getT();
//        rotationMatrices.push_back(pR);
//        translationMatrices.push_back(pT);
//    }
//    // right cameras
//    for(int i = 0; i < nFrames; i++)
//    {
//        Mat relativeT = Mat::zeros(3, 1, CV_64F);
//        relativeT.at<double>(0, 0) = 0.537;
//        Mat pR;
//        transpose(views[i]->getR(), pR);
//        Mat pT = -pR * ((views[i]->getR() * relativeT + views[i]->getT()));
//        
//        rotationMatrices.push_back(pR);
//        translationMatrices.push_back(pT);
//    }
//    tCopy = translationMatrices;
//    Canvas *canvas = new Canvas();
//
////    canvas->drawFeatureMatches(views[v1]->getImgs()[0], views[v2]->getImgs()[0],
////                               ps1, ps2);
//    // reprojection
//    
//    // now bundle adjust!!!
//    
//    // do not take first view
//    vector<vector<Point2d>> ba2DPoints;
//    ba2DPoints.insert(ba2DPoints.end(), leftFeatureTrack.begin() + 1, leftFeatureTrack.end());
//    ba2DPoints.insert(ba2DPoints.end(), rightFeatureTrack.begin() + 1, rightFeatureTrack.end());
//    vector<vector<int>> baVisibility((nFrames - 1) * 2, vector<int>(survivingFids.size(), 1));
//    vector<Mat> baCamMatr,baRotMatr, baTrMatr;
//    baCamMatr = vector<Mat>(cameraMatrices.begin() + 2, cameraMatrices.end());
//    baRotMatr.insert(baRotMatr.end(), rotationMatrices.begin() + 1, rotationMatrices.begin() + nFrames);
//    baRotMatr.insert(baRotMatr.end(), rotationMatrices.begin() + nFrames + 1, rotationMatrices.end());
//    baTrMatr.insert(baTrMatr.end(), translationMatrices.begin() + 1, translationMatrices.begin() + nFrames);
//    baTrMatr.insert(baTrMatr.end(), translationMatrices.begin() + nFrames + 1, translationMatrices.end());
//    vector<Mat> baDistCoefficients((nFrames - 1)* 2, Mat::zeros(5, 1, CV_64F));
//
//                    
//
//    cvsba::Sba::Params params ;
//    params.type = Sba::MOTIONSTRUCTURE;
//    params.fixedIntrinsics = 5;
//    params.fixedDistortion = 5;
//    params.iterations = 1;
//    params.verbose = false;
//    sba.setParams(params);
//    
//    double sbaError = 0.0;
//    double prevSbaError = 0.0;
//    double threshold = 0.02;
//    
//    for(int i = 0; i < 100; i++)
//    {
//        sbaError =  sba.run(points3D, ba2DPoints, baVisibility, baCamMatr, baRotMatr, baTrMatr, baDistCoefficients);
//        if(prevSbaError != 0 && abs(sbaError - prevSbaError) / prevSbaError < threshold)
//        {
//            break;
//        }
//        prevSbaError = sbaError;
//        if(i == 99)
//            cout << "Iteration limit hit!" << endl;
//    }
//    
//    if(sbaError > 500)
//    {
//        cout << "abandoned." << endl;
//    }
//    else
//    {
//        for(int i = 1; i < nFrames; i++)
//        {
//            views[i]->setPose(baRotMatr[i - 1].inv(), -baRotMatr[i - 1].inv() * baTrMatr[i - 1]);
//        }
//    }
//    cout << "BA finished." << endl;
}

set<int> ViewTracker::findSurvivingFids()
{
    set<int> survivingFids;
//    // find surviving fids from last view
//    View* lastView = getCurrView();
//    map<int, int> idMap = lastView->getFids();
//    for(map<int, int>::iterator idIter = idMap.begin(); idIter != idMap.end(); idIter++)
//        survivingFids.insert(idIter->first);
    return survivingFids;
}


