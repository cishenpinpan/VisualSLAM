//
//  PoseEstimator.cpp
//  VisualSLAM
//
//  Created by Rong Yuan on 3/4/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#include "PoseEstimator.h"

PoseEstimator::PoseEstimator()
{
    featureExtractor = new FeatureExtractor();
    featureTracker = new FeatureTracker();
}
void evaluateReprojectionError(const double *par, const int nEqs, const void *data, double *fvec, int *userBreak)
{    
    double lambda = *par;
    // take a look at data
    DataStruct *dataStruct = (DataStruct*) data;
    // 1. two views
    View* v1 = dataStruct->v1, *v2 = dataStruct->v2;
    // 2. 3D points
    vector<Point3d> points3D = dataStruct->points3D;
    // 3. R and T
    Mat R = dataStruct->R.clone(), T = dataStruct->T.clone();
    Mat scaledT = T * lambda;
    
    Mat poseChange = Mat::eye(4, 4, CV_64F);
    Mat aux = poseChange(Rect(0, 0 ,3, 3));
    R.copyTo(aux);
    aux = poseChange(Rect(3, 0, 1, 3));
    scaledT.copyTo(aux);
    
    /***  
    // uncomment this section to cheat using ground truth
     
    Mat cheatedPose = v1->getGroundTruth().inv() * v2->getGroundTruth();
    Mat cheatedR = cheatedPose(Rect(0, 0, 3, 3));
    Mat cheatedT = cheatedPose(Rect(3, 0, 1, 3));
    cheatedT.at<double>(0, 0) = cheatedT.at<double>(0, 0) / cheatedT.at<double>(2, 0);
    cheatedT.at<double>(1, 0) = cheatedT.at<double>(1, 0) / cheatedT.at<double>(2, 0);
    cheatedT = lambda * cheatedT;
    
    Mat cheatedT(3, 1, CV_64F);
    cheatedT.at<double>(0, 0) = -0.0033 / 0.6839;
    cheatedT.at<double>(1, 0) = -0.0070 / 0.6839;
    cheatedT.at<double>(2, 0) = 0.6839 / 0.6839;
    cheatedT = lambda * cheatedT;
    cheatedT.copyTo(aux);
     
    **/

    Mat nextLeftPose = v1->getPose() * poseChange.clone();
    Mat nextRightPose = nextLeftPose * CameraParameters::getStereoPose();
    vector<Point2f> nextLeftFeatures = v2->getLeftFeatureSet().getFeaturePoints(),
                    nextRightFeatures = v2->getRightFeatureSet().getFeaturePoints();
    // go through each point
    double error = 0.0;
    double errorsHist[4] = {0.0, 0.0, 0.0, 0.0};
    int histCount[4] = {0, 0, 0, 0};
    int errorsCounted = 0;
    for(int i = 0; i < points3D.size(); i++)
    {
        Point3d point3d = points3D[i];
        Mat temp(3, 1, CV_64F);
        temp = Mat(point3d);
        Mat gammaC = (v1->getR().inv() * (temp - v1->getT()));
        temp.release();
        double depth = gammaC.at<double>(2, 0);
        // reproject onto both two views and compute the reprojection error (not squared)
        pair<double, double> err = {0.0, 0.0};
        double errThreshold = 5;
        error = 0.0;
        // left
        if(depth > 0 && depth < 100)
            err = reproject3DPoint(point3d, nextLeftPose, nextLeftFeatures[i], false);
        
        err.first = abs(err.first) < errThreshold ? err.first : 0;
        err.second = abs(err.second) < errThreshold ? err.second : 0;
        fvec[4 * i] = err.first;
        fvec[4 * i + 1] = err.second;
        error += (err.first * err.first + err.second * err.second);
        errorsCounted = error == 0 ? errorsCounted + 1 : errorsCounted;
        
        // histogram of errors (vs depth)
        if(error != 0)
        {
            if(depth <= 10)
            {
                ++histCount[0];
                errorsHist[0] += error;
            }
            else if(depth <= 30)
            {
                ++histCount[1];
                errorsHist[1] += error;
            }
            else if(depth < 50)
            {
                ++histCount[2];
                errorsHist[2] += error;
            }
            else if(depth >= 50)
            {
                ++histCount[3];
                errorsHist[3] += error;
            }
            else
            {
                cout << "negative depth wtf!" << endl;
            }
        }
        
        // right
        if(depth > 0 && depth < 100)
            err = reproject3DPoint(point3d, nextRightPose, nextRightFeatures[i], false);
        
        err.first = abs(err.first) < errThreshold ? err.first : 0;
        err.second = abs(err.second) < errThreshold ? err.second : 0;
        fvec[4 * i + 2] = err.first;
        fvec[4 * i + 3] = err.second;
        error += (err.first * err.first + err.second * err.second);
        errorsCounted = error == 0 ? errorsCounted + 1 : errorsCounted;
        
        if(error != 0)
        {
            if(depth <= 10)
            {
                ++histCount[0];
                errorsHist[0] += error;
            }
            else if(depth <= 30)
            {
                ++histCount[1];
                errorsHist[1] += error;
            }
            else if(depth < 50)
            {
                ++histCount[2];
                errorsHist[2] += error;
            }
            else if(depth >= 50)
            {
                ++histCount[3];
                errorsHist[3] += error;
            }
            else
            {
                cout << "negative depth wtf!" << endl;
            }
        }
        
        
    }
    double iterations = dataStruct->iterations;
    dataStruct->iterations = iterations + 1;
    
    /***
    // uncooment this section to display l1 norm error distribution over feature depth
     
    for(int i = 0; i < 4; i++)
    {
        cout << sqrt(errorsHist[i] / histCount[i]) << "(" << histCount[i] << ") ";
    }
    cout << "-------end(" << *par << ")--------" ;
    cout << endl;
     
     **/
     
}
     

double PoseEstimator::estimateScale(View *v1, View *v2)
{
    // compute relative R and T first
    Mat relativePose = v1->getPose().inv() * v2->getPose();
    Mat R = relativePose(Rect(0, 0, 3, 3));
    Mat T = relativePose(Rect(3, 0, 1, 3));
    double lambda[1];
    lambda[0] = 1;

    double *par = lambda;
    
    // 1. triangulate 2d features from v1 to 3d world
    vector<Point3d> points3D;
    Mat stereoPose = CameraParameters::getStereoPose();
    triangulatePoints(v1->getPose(), stereoPose.clone(),
                      v1->getLeftFeatureSet().getFeaturePoints(),
                      v1->getRightFeatureSet().getFeaturePoints(), points3D);
    const int nEqs = int(points3D.size()) * 4;
    DataStruct dataStruct(v1, v2, R.clone(), T.clone(), points3D);
    // 2. nonlinear optimization (levenburg marquardt)
    /* auxiliary parameters */
    lm_control_struct control = lm_control_double;
    lm_status_struct  status;
    
     control.verbosity = 0;
    
//    lmmin(const int n_par, <#double *par#>, <#const int m_dat#>, <#const void *data#>, void (*evaluate)(const double *, const int, const void *, double *, int *), <#const lm_control_struct *control#>, lm_status_struct *status)
    lmmin(1, par, nEqs, &dataStruct, evaluateReprojectionError, &control, &status);
    
    return *par;
}

Mat PoseEstimator::estimatePose(View *v1, View *v2)
{
    return estimatePoseMono(v1, v2);
}

Mat PoseEstimator::estimatePoseMono(View *v1, View *v2)
{
    // 1. assume v1 has feature extracted but not v2
    
    // 2. track to left image of view 2
    Mat featureTrackingStat = featureTracker->track(v1->getImgs()[0], v2->getImgs()[0],
                                        v1->getLeftFeatureSet(),v2->getLeftFeatureSet(), false);
    // reject outliers
    rejectOutliers(v1, v2, featureTrackingStat);
    
    // 3. estimate essential matrix
    Mat essentialMatrixStat;
    
    Mat E = findEssentialMat(v2->getLeftFeatureSet().getFeaturePoints(), v1->getLeftFeatureSet().getFeaturePoints(),
                        CameraParameters::focal, CameraParameters::principal, RANSAC, 0.999, 1.0, essentialMatrixStat);
    // reject outliers
    rejectOutliers(v1, v2, essentialMatrixStat);

    // cout << "essential matrix inliers: " << count << "/" << points1.size() << endl;
    
    // 4. recorver pose (translation up to a scale)
    Mat poseRecoveryStat;
    Mat R, t;
    recoverPose(E, v2->getLeftFeatureSet().getFeaturePoints(), v1->getLeftFeatureSet().getFeaturePoints(), R, t,
                CameraParameters::focal, CameraParameters::principal, poseRecoveryStat);
    
    // reject outliers
    rejectOutliers(v1, v2, poseRecoveryStat);
    
    // cout << "recovering pose inliers: " << count << "/" << points1.size()<< endl;
    
    // 6. estimate scale (later)
    Mat poseChange = Mat::eye(4, 4, CV_64F);
    Mat aux = poseChange(Rect(0, 0, 3, 3));
    R.copyTo(aux);
    aux = poseChange(Rect(3, 0, 1, 3));
    t.copyTo(aux);
    v2->setPose(v1->getPose() * poseChange);
    
    // 7. return pose
    return poseChange.clone();
}

void PoseEstimator::rejectOutliers(View *v1, View *v2, Mat status)
{
    map<int, int> fids;
    vector<Point2f> points1, points2;
    vector<long> ids1, ids2;
    map<long ,int> idBook1, idBook2;
    
    for(int i = 0 ; i < status.rows; i++)
    {
        if(status.at<bool>(i, 0) == 1)
        {
            Feature f1 = v1->getLeftFeatureSet().getFeatureByIndex(i),
            f2 = v2->getLeftFeatureSet().getFeatureByIndex(i);
            points1.push_back(f1.getPoint());
            ids1.push_back(f1.getId());
            idBook1.insert(make_pair(f1.getId(), int(points1.size() - 1)));
            points2.push_back(f2.getPoint());
            ids2.push_back(f2.getId());
            idBook2.insert(make_pair(f2.getId(), int(points2.size() - 1)));
        }
    }
    v1->getLeftFeatureSet().setFeaturePoints(points1);
    v1->getLeftFeatureSet().setIds(ids1);
    v1->setIdBook(idBook1);
    v2->getLeftFeatureSet().setFeaturePoints(points2);
    v2->getLeftFeatureSet().setIds(ids2);
    v2->setIdBook(idBook2);
    points1.clear();
    points2.clear();
    ids1.clear();
    ids2.clear();
    idBook1.clear();
    idBook2.clear();
}
