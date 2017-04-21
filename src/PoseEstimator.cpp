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
    // unpack data
    ScaleEstimatorStruct *dataStruct = (ScaleEstimatorStruct*) data;
    vector<KeyPoint> keyPoints = dataStruct->keyPoints;
    vector<Point3d> p3ds = dataStruct->landmarks;
    View *v = dataStruct->v;
    const Mat prevPose = dataStruct->prevPose;
    
    // update pose
    Mat relativePose = prevPose.inv() * v->getPose();
    // multiply by lambda
    relativePose.col(3).rowRange(0, 3) = lambda * relativePose.col(3).rowRange(0, 3);
    const Mat currPose = prevPose * relativePose;
    
    double totalError = 0.0;
    for(int i = 0; i < p3ds.size(); i++)
    {
        Point3d p3d = p3ds[i];
        KeyPoint kp = keyPoints[i];
        pair<double, double> err = reproject3DPoint(p3d, currPose, kp, false);
        *(fvec++) = err.first;
        *(fvec++) = err.second;
        double reprojectionError = sqrt(err.first * err.first + err.second * err.second);
        totalError += reprojectionError;
    }
    cout << "total error: " << totalError << endl;
     
}
     

Mat PoseEstimator::solveScalePnP(View *v, const Mat prevPose, map<long, Landmark> landmarkBook, double initial)
{
    // variable
    double lambda;

    double *par = &lambda;
    
    // 1. obtain corresponding landmarks
    vector<Point3d> p3ds;
    vector<KeyPoint> kps;
    for(int i = 0; i < v->getLeftFeatureSet().size(); i++)
    {
        KeyPoint kp = v->getLeftFeatureSet().getFeaturePoints()[i];
        long id = v->getLeftFeatureSet().getIds()[i];
        if(!landmarkBook.count(id))
            continue;
        p3ds.push_back(landmarkBook[id].getPoint());
        kps.push_back(kp);
    }
    
    const int nEqs = int(p3ds.size()) * 2;
    ScaleEstimatorStruct dataStruct(v, prevPose, kps, p3ds);
    // 2. nonlinear optimization (levenburg marquardt)
    /* auxiliary parameters */
    lm_control_struct control = lm_control_double;
    lm_status_struct  status;
    
    control.verbosity = 0;
    
    lmmin(1, par, nEqs, &dataStruct, evaluateReprojectionError, &control, &status);
    
    // update pose
    Mat relativePose = prevPose.inv() * v->getPose();
    // multiply by lambda
    relativePose.col(3).rowRange(0, 3) = lambda * relativePose.col(3).rowRange(0, 3);
    const Mat currPose = prevPose * relativePose;
    v->setPose(currPose);
    
    return v->getPose();
}

void computeReprojectionErrorByTrinocular
(const double *par, const int nEqs, const void *data, double *fvec, int *userBreak)
{
    // unpack initial guess (of the pose)
    double r1 = par[0], r2 = par[1], r3 = par[2], t1 = par[3], t2 = par[4], t3 = par[5];
    Mat rVec(3, 1, CV_64F), tVec(3, 1, CV_64F);
    rVec.at<double>(0, 0) = r1;
    rVec.at<double>(1, 0) = r2;
    rVec.at<double>(2, 0) = r3;
    tVec.at<double>(0, 0) = t1;
    tVec.at<double>(1, 0) = t2;
    tVec.at<double>(2, 0) = t3;
    Mat R;
    Rodrigues(rVec, R);
    Mat t = tVec.clone();
    Mat pose_v = Converter::rotationTranslationToPose(R, t);
    // unpack data struct
    TrinocularDataStruct *dataStruct = (TrinocularDataStruct*) data;
    View *v1 = dataStruct->v1;
    View *v2 = dataStruct->v2;
    View *v = dataStruct->v;
    map<long, vector<KeyPoint>> commonFeatures(dataStruct->commonFeatures);
    
    // compute essential matrices
    //    const Mat pose_v1v = v1->getPose().inv() * pose_v;
    //    const Mat pose_v2v = v2->getPose().inv() * pose_v;
    //    const Mat R_v1v = pose_v1v.rowRange(0, 3).colRange(0, 3).clone();
    //    const Mat R_v2v = pose_v2v.rowRange(0, 3).colRange(0, 3).clone();
    //    const Mat t_v1v = pose_v1v.rowRange(0, 3).col(3).clone();
    //    const Mat t_v2v = pose_v2v.rowRange(0, 3).col(3).clone();
    //    const Mat t_v1v_x = Converter::tVecToTx(t_v1v);
    //    const Mat t_v2v_x = Converter::tVecToTx(t_v2v);
    //    const Mat E_v1v = t_v1v_x * R_v1v;
    //    const Mat E_v2v = t_v2v_x * R_v2v;
    
    // double compute reprojection error
    Mat pose_v1v = v1->getPose().inv() * pose_v;
    Mat pose_v2v = v2->getPose().inv() * pose_v;
    pose_v1v = pose_v1v.inv();
    pose_v2v = pose_v2v.inv();
    Mat R_v1v = pose_v1v.rowRange(0, 3).colRange(0, 3).clone();
    Mat R_v2v = pose_v2v.rowRange(0, 3).colRange(0, 3).clone();
    Mat t_v1v = pose_v1v.rowRange(0, 3).col(3).clone();
    Mat t_v2v = pose_v2v.rowRange(0, 3).col(3).clone();
    Mat t_v1v_x = Converter::tVecToTx(t_v1v);
    Mat t_v2v_x = Converter::tVecToTx(t_v2v);
    Mat E_v1v = t_v1v_x * R_v1v;
    Mat E_v2v = t_v2v_x * R_v2v;
    double totalError = 0.0;
    int i = 0;
    for(map<long, vector<KeyPoint>>::iterator it = commonFeatures.begin(); it != commonFeatures.end(); it++)
    {
        
        KeyPoint kp_v1 = it->second[0], kp_v2 = it->second[1], kp_v = it->second[2];
        Point3d p3d;
        triangulatePoint(v1->getPose(), v2->getPose(), kp_v1, kp_v2, p3d);
        pair<double, double> err = reproject3DPoint(p3d, pose_v, kp_v, false);
        fvec[i++] = err.first;
        fvec[i++] = err.second;
        totalError += (err.first * err.first + err.second * err.second);
        // 9754 12829 15240
        Canvas canvas;
        vector<Point2f> ps_v1, ps_v2, ps_v;
        ps_v1.push_back(Point2f(kp_v1.pt.x, kp_v1.pt.y));
        ps_v2.push_back(Point2f(kp_v2.pt.x, kp_v2.pt.y));
        ps_v.push_back(Point2f(kp_v.pt.x, kp_v.pt.y));
//        Mat p_v1 = Mat::ones(3, 1, CV_64F), p_v2 = Mat::ones(3, 1, CV_64F), p_v = Mat::ones(3, 1, CV_64F);
//        p_v1.at<double>(0, 0) = kp_v1.pt.x;
//        p_v1.at<double>(1, 0) = kp_v1.pt.y;
//        p_v2.at<double>(0, 0) = kp_v2.pt.x;
//        p_v2.at<double>(1, 0) = kp_v2.pt.y;
//        p_v.at<double>(0, 0) = kp_v.pt.x;
//        p_v.at<double>(1, 0) = kp_v.pt.y;
//        p_v1 = CameraParameters::getIntrinsic().inv() * p_v1.clone();
//        p_v2 = CameraParameters::getIntrinsic().inv() * p_v2.clone();
//        p_v = CameraParameters::getIntrinsic().inv() * p_v.clone();
//        const double x = p_v.at<double>(0, 0), y = p_v.at<double>(1, 0);
//        const Mat epi_v1v = E_v1v * p_v1, epi_v2v = E_v2v * p_v2;
//        double a1 = epi_v1v.at<double>(0, 0), b1 = epi_v1v.at<double>(1, 0), c1 = epi_v1v.at<double>(2, 0);
//        double a2 = epi_v2v.at<double>(0, 0), b2 = epi_v2v.at<double>(1, 0), c2 = epi_v2v.at<double>(2, 0);
//        // compute intersection of two epipolar lines (reprojection point)
//        double x_hat = (b1 * c2 - b2 * c1) / (a1 * b2 - a2 * b1);
//        double y_hat = (a2 * c1 - a1 * c2) / (a1 * b2 - a2 * b1);
//        Mat projected_hat(3, 1, CV_64F);
//        projected_hat.at<double>(0, 0) = x_hat;
//        projected_hat.at<double>(1, 0) = y_hat;
//        projected_hat.at<double>(2, 0) = 1;
//        projected_hat = CameraParameters::getIntrinsic() * projected_hat;
//        cout << projected_hat << endl;
//        Mat projected(3, 1, CV_64F);
//        projected.at<double>(0, 0) = x;
//        projected.at<double>(1, 0) = y;
//        projected.at<double>(2, 0) = 1;
//        projected = CameraParameters::getIntrinsic() * projected;
//        cout << projected << endl;
//        cout << "----------" << endl;
//        fvec[i++] = projected.at<double>(0, 0) - projected_hat.at<double>(0, 0);
//        fvec[i++] = projected.at<double>(1, 0) - projected_hat.at<double>(1, 0);
        // cout << "err: ";
        // cout << CameraParameters::focal * (x - x_hat) << ", " << CameraParameters::focal * (y - y_hat) << endl;
    }
}
Mat PoseEstimator::estimatePoseByTrinocular(View *v1, View *v2, View *v)
{
    // find common feature points
    map<long, vector<KeyPoint>> commonFeatures;
    FeatureSet featureSet_v1 = v1->getLeftFeatureSet();
    FeatureSet featureSet_v2 = v2->getLeftFeatureSet();
    FeatureSet featureSet_v = v->getLeftFeatureSet();
    for(int i = 0; i < featureSet_v1.size(); i++)
    {
        long id = featureSet_v1.getIds()[i];
        commonFeatures[id].push_back(featureSet_v1.getFeaturePoints()[i]);
    }
    for(int i = 0; i < featureSet_v2.size(); i++)
    {
        long id = featureSet_v2.getIds()[i];
        commonFeatures[id].push_back(featureSet_v2.getFeaturePoints()[i]);
    }
    for(int i = 0; i < featureSet_v.size(); i++)
    {
        long id = featureSet_v.getIds()[i];
        commonFeatures[id].push_back(featureSet_v.getFeaturePoints()[i]);
    }
    // remove non-common feature points
    for(map<long, vector<KeyPoint>>::iterator it = commonFeatures.begin(); it != commonFeatures.end(); )
    {
        if(it->second.size() < 3)
            it = commonFeatures.erase(it);
        else
            it++;
    }
    // 6 DoF
    double lambda[6] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    
    double *par = lambda;

    TrinocularDataStruct dataStruct(v1, v2, v, commonFeatures, 0, 10);
    const int nEqs = int(commonFeatures.size()) * 2;
    // 2. nonlinear optimization (levenburg marquardt)
    /* auxiliary parameters */
    lm_control_struct control = lm_control_double;
    lm_status_struct  status;
    
    control.verbosity = 0;
    
    //    lmmin(const int n_par, <#double *par#>, const int m_dat, <#const void *data#>, void (*evaluate)(const double *, const int, const void *, double *, int *), <#const lm_control_struct *control#>, lm_status_struct *status)
    
    lmmin(6, par, nEqs, &dataStruct, computeReprojectionErrorByTrinocular, &control, &status);
    
    // re-construct pose
    Mat rVec = Mat::zeros(3, 1, CV_64F), t = Mat::zeros(3, 1, CV_64F);
    rVec.at<double>(0, 0) = par[0];
    rVec.at<double>(1, 0) = par[1];
    rVec.at<double>(2, 0) = par[2];
    Mat R;
    Rodrigues(rVec, R);
    t.at<double>(0, 0) = par[3];
    t.at<double>(1, 0) = par[4];
    t.at<double>(2, 0) = par[5];
    Mat pose = Converter::rotationTranslationToPose(R, t);
    v->setPose(pose);
    return pose.clone();
}
Mat PoseEstimator::estimatePose(View *v1, View *v2, double lambda)
{
    return estimatePoseMono(v1, v2, lambda);
}

Mat PoseEstimator::estimatePoseMono(View *v1, View *v2, double lambda)
{
    // 1. assume features are not matched
    vector<KeyPoint> newKps1, newKps2;
    vector<long> prevIds = v2->getLeftFeatureSet().getIds(), newIds;
    for(int i = 0; i < prevIds.size(); i++)
    {
        long id = prevIds[i];
        KeyPoint kp1 = v1->getLeftFeatureSet().getFeatureById(id).getPoint();
        KeyPoint kp2 = v2->getLeftFeatureSet().getFeaturePoints()[i];
        newKps1.push_back(kp1);
        newKps2.push_back(kp2);
        newIds.push_back(id);
    }
    v1->getLeftFeatureSet().setFeaturePoints(newKps1);
    v1->getLeftFeatureSet().setIds(newIds);
    v2->getLeftFeatureSet().setFeaturePoints(newKps2);
    v2->getLeftFeatureSet().setIds(newIds);
    // 2. estimate essential matrix
    Mat essentialMatrixStat;
    
    vector<Point2f> points1, points2;
    KeyPoint::convert(v1->getLeftFeatureSet().getFeaturePoints(), points1);
    KeyPoint::convert(v2->getLeftFeatureSet().getFeaturePoints(), points2);
    Mat E = findEssentialMat(points2, points1,
                        CameraParameters::focal, CameraParameters::principal, RANSAC, 0.999, 1.0, essentialMatrixStat);
    // reject outliers
    rejectOutliers(v1, v2, essentialMatrixStat);

    // cout << "essential matrix inliers: " << count << "/" << points1.size() << endl;
    
    // 3. recorver pose (translation up to a scale)
    Mat poseRecoveryStat;
    Mat R, t;
    KeyPoint::convert(v1->getLeftFeatureSet().getFeaturePoints(), points1);
    KeyPoint::convert(v2->getLeftFeatureSet().getFeaturePoints(), points2);
    recoverPose(E, points2, points1, R, t,
                CameraParameters::focal, CameraParameters::principal, poseRecoveryStat);
    t = lambda * t;
    // reject outliers
    rejectOutliers(v1, v2, poseRecoveryStat);
    
    // cout << "recovering pose inliers: " << count << "/" << points1.size()<< endl;
    
    // 4. update pose
    Mat poseChange = Mat::eye(4, 4, CV_64F);
    Mat aux = poseChange(Rect(0, 0, 3, 3));
    R.copyTo(aux);
    aux = poseChange(Rect(3, 0, 1, 3));
    t.copyTo(aux);
    v2->setPose(v1->getPose() * poseChange);
    
    // 7. return pose
    return poseChange.clone();
}
double PoseEstimator::solvePnP(View *v, map<long, Landmark> &landmarkBook)
{
    vector<Point3d> landmarks;
    vector<Point2f> imagePoints;
    const vector<long> ids = v->getLeftFeatureSet().getIds();
    KeyPoint::convert(v->getLeftFeatureSet().getFeaturePoints(), imagePoints);
    for(int i = 0; i < imagePoints.size(); i++)
    {
        // imagePoints[i].y = 376 - imagePoints[i].y;
    }
    for(long id : ids)
    {
        landmarks.push_back(landmarkBook[id].getPoint());
    }
    const Mat cameraMatrix = CameraParameters::getIntrinsic();
    const Mat distCoeffs = CameraParameters::getDistCoeff();
    Mat R, Rvec, t;
    Mat status;
    solvePnPRansac(landmarks, imagePoints, cameraMatrix, distCoeffs, Rvec, t, false, 100, 5.0, 0.99, status, CV_EPNP);
    Rodrigues(Rvec, R);
    Mat pose = Converter::rotationTranslationToPose(R, t);
    pose = pose.inv();
    v->setPose(pose);
    // reject outliers
    vector<KeyPoint> newFeatures;
    vector<long> newIds;
    for(int i = 0; i < status.rows; i++)
    {
        KeyPoint kp = v->getLeftFeatureSet().getFeaturePoints()[i];
        long id = v->getLeftFeatureSet().getIds()[i];
        // reprojection error
        pair<double, double> err = reproject3DPoint(landmarkBook[id].getPoint(), v->getPose(), kp, false);
        double reprojectionErr = sqrt(err.first * err.first + err.second * err.second);
        if(reprojectionErr > 5.0)
            continue;
        newFeatures.push_back(kp);
        newIds.push_back(id);
    }
    double inliers = newFeatures.size() * 1.0 / v->getLeftFeatureSet().size();
    if(inliers < 0.0)
        return inliers;
    v->getLeftFeatureSet().setFeaturePoints(newFeatures);
    v->getLeftFeatureSet().setIds(newIds);
    return inliers;
}
void computeReprojectionErrorFourFrames(const void *data, const double lambda, double *fvec)
{
    double inlierRatio = 0.0;
    double reprojectionError = 0.0;
    // unpack data struct
    ScaleRefinementDataStruct *dataStruct = (ScaleRefinementDataStruct*) data;
    View *v1 = dataStruct->v1;
    View *v2 = dataStruct->v2;
    View *v3 = dataStruct->v3;
    View *v4 = dataStruct->v4;
    map<long, vector<KeyPoint>> commonFeatures(dataStruct->commonFeatures);
    map<long, Landmark> landmarkBook(dataStruct->landmarkBook);
    // compute relative poses
    Mat pose12 = v1->getPose().inv() * v2->getPose();
    Mat pose23 = v2->getPose().inv() * v3->getPose();
    Mat pose34 = v3->getPose().inv() * v4->getPose();
    // multiply pose23 by lambda
    pose23.col(3).rowRange(0, 3) = pose23.col(3).rowRange(0, 3) * lambda;
    // compute adjusted poses
    Mat pose_v1 = v1->getPose();
    Mat pose_v2 = v2->getPose();
    Mat pose_v3 = pose_v2 * pose23;
    Mat pose_v4 = pose_v3 * pose34;
    // compute re-projection errors
    double totalError = 0.0;
    int inlierCount = 0, totalCount = 0;
    for(map<long, vector<KeyPoint>>::iterator it = commonFeatures.begin(); it != commonFeatures.end(); it++)
    {
        bool isInlier = true;
        totalCount++;
        long id = it->first;
        KeyPoint kp_v1 = it->second[0], kp_v2 = it->second[1], kp_v3 = it->second[2], kp_v4 = it->second[3];
        Point3d p3d = landmarkBook[id].getPoint();
        pair<double, double> err = {0.0, 0.0};
        err = reproject3DPoint(p3d, pose_v1, kp_v1, false);
        if(sqrt(err.first * err.first + err.second * err.second) > 5.0)
            isInlier = false;
        if(fvec != NULL)
        {
            *(fvec++) = err.first;
            *(fvec++) = err.second;
        }
        totalError += (err.first * err.first + err.second * err.second);
        err = reproject3DPoint(p3d, pose_v2, kp_v2, false);
        if(sqrt(err.first * err.first + err.second * err.second) > 5.0)
            isInlier = false;
        if(fvec != NULL)
        {
            *(fvec++) = err.first;
            *(fvec++) = err.second;
        }
        totalError += (err.first * err.first + err.second * err.second);
        err = reproject3DPoint(p3d, pose_v3, kp_v3, false);
        if(sqrt(err.first * err.first + err.second * err.second) > 5.0)
            isInlier = false;
        if(fvec != NULL)
        {
            *(fvec++) = err.first;
            *(fvec++) = err.second;
        }
        totalError += (err.first * err.first + err.second * err.second);
        err = reproject3DPoint(p3d, pose_v4, kp_v4, false);
        if(sqrt(err.first * err.first + err.second * err.second) > 5.0)
            isInlier = false;
        if(fvec != NULL)
        {
            *(fvec++) = err.first;
            *(fvec++) = err.second;
        }
        totalError += (err.first * err.first + err.second * err.second);
        if(isInlier)
            inlierCount++;
    }
    inlierRatio = double(inlierCount) / totalCount;
    dataStruct->inlier = inlierRatio;
    dataStruct->error = totalError;
}
void computeReprojectionErrorFourFrames
(const double *par, const int nEqs, const void *data, double *fvec, int *userBreak)
{
    // unpack initial guess (of the pose)
    double lambda = *par;
    // compute reprojection error
    computeReprojectionErrorFourFrames(data, lambda, fvec);
}
void PoseEstimator::refineScale(vector<View*> views, map<long, Landmark> &landmarkBook)
{
    // unpack 4 frames
    View *v1 = views[0], *v2 = views[1], *v3 = views[2], *v4 = views[3];
    // find common features in four frames
    map<long, vector<KeyPoint>> commonFeatures;
    FeatureSet featureSet_v1 = v1->getLeftFeatureSet();
    FeatureSet featureSet_v2 = v2->getLeftFeatureSet();
    FeatureSet featureSet_v3 = v3->getLeftFeatureSet();
    FeatureSet featureSet_v4 = v4->getLeftFeatureSet();
    for(int i = 0; i < featureSet_v1.size(); i++)
    {
        long id = featureSet_v1.getIds()[i];
        commonFeatures[id].push_back(featureSet_v1.getFeaturePoints()[i]);
    }
    for(int i = 0; i < featureSet_v2.size(); i++)
    {
        long id = featureSet_v2.getIds()[i];
        commonFeatures[id].push_back(featureSet_v2.getFeaturePoints()[i]);
    }
    for(int i = 0; i < featureSet_v3.size(); i++)
    {
        long id = featureSet_v3.getIds()[i];
        commonFeatures[id].push_back(featureSet_v3.getFeaturePoints()[i]);
    }
    for(int i = 0; i < featureSet_v4.size(); i++)
    {
        long id = featureSet_v4.getIds()[i];
        commonFeatures[id].push_back(featureSet_v4.getFeaturePoints()[i]);
    }
    // remove non-common feature points
    // record ids
    vector<long> ids;
    for(map<long, vector<KeyPoint>>::iterator it = commonFeatures.begin(); it != commonFeatures.end(); )
    {
        if(it->second.size() < 4)
            it = commonFeatures.erase(it);
        else
        {
            ids.push_back(it->first);
            it++;
        }
    }
    if(commonFeatures.size() < 20)
        return ;
    // RANSAC
    srand(time(NULL));
    double maxInlierRatio = 0.0;
    double bestLambda = 1.0;
    for(int i = 0; i < 20; i++)
    {
        // generate k random number between 0 and commonFeatures.size()
        map<long, vector<KeyPoint>> commonFeaturesRANSAC;
        for(int k = 0; k < 5; k++)
        {
            long randomId = ids[rand() % commonFeatures.size()];
            while(commonFeaturesRANSAC.count(randomId))
                randomId = ids[rand() % commonFeatures.size()];
            commonFeaturesRANSAC[randomId] = vector<KeyPoint>(commonFeatures[randomId]);
        }
        // lm
        // 1 DoF
        double lambda = .99;
        
        double *par = &lambda;
        
        ScaleRefinementDataStruct *dataStruct =
                new ScaleRefinementDataStruct(v1, v2, v3, v4, landmarkBook, commonFeaturesRANSAC);
        const int nEqs = int(commonFeaturesRANSAC.size()) * 4 * 2;
        // 2. nonlinear optimization (levenburg marquardt)
        /* auxiliary parameters */
        lm_control_struct control = lm_control_double;
        lm_status_struct  status;
        
        control.verbosity = 0;
        
        //    lmmin(const int n_par, <#double *par#>, const int m_dat, <#const void *data#>, void (*evaluate)(const double *, const int, const void *, double *, int *), <#const lm_control_struct *control#>, lm_status_struct *status)
        
        lmmin(1, par, nEqs, dataStruct, computeReprojectionErrorFourFrames, &control, &status);
        
        
        // test this lambda on entire dataset
        ScaleRefinementDataStruct *completeDataStruct =
        new ScaleRefinementDataStruct(v1, v2, v3, v4, landmarkBook, commonFeatures);
        computeReprojectionErrorFourFrames(completeDataStruct, lambda, NULL);
        // record only the best set of data points and their corresponding ratio
        double inlierRatio = dataStruct->inlier;
        if(maxInlierRatio < inlierRatio)
        {
            maxInlierRatio = inlierRatio;
            bestLambda = lambda;
        }
    }
    
    
    // update poses
    // compute relative poses
    Mat pose12 = v1->getPose().inv() * v2->getPose();
    Mat pose23 = v2->getPose().inv() * v3->getPose();
    Mat pose34 = v3->getPose().inv() * v4->getPose();
    // multiply pose23 by lambda
    double lambda = bestLambda;
    if(abs(lambda - 1.0) > 0.20 || maxInlierRatio < 0.5)
    {
        cout << "Failed!" << endl;
        return ;
    }

    pose23.col(3).rowRange(0, 3) = pose23.col(3).rowRange(0, 3) * lambda;
    // compute adjusted poses
    Mat pose_v1 = v1->getPose();
    Mat pose_v2 = v2->getPose();
    Mat pose_v3 = pose_v2 * pose23;
    Mat pose_v4 = pose_v3 * pose34;
    v1->setPose(pose_v1);
    v2->setPose(pose_v2);
    v3->setPose(pose_v3);
    v4->setPose(pose_v4);
    
    cout << "lambda: " << lambda << endl;
    cout << "inliers: " << maxInlierRatio * 100 << "%" << endl;
}
void PoseEstimator::rejectOutliers(View *v1, View *v2, Mat status)
{
    map<int, int> fids;
    vector<KeyPoint> points1, points2;
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
