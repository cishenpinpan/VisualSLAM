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
    vector<KeyPoint> nextLeftFeatures = v2->getLeftFeatureSet().getFeaturePoints(),
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
    
//    lmmin(const int n_par, <#double *par#>, const int m_dat, <#const void *data#>, void (*evaluate)(const double *, const int, const void *, double *, int *), <#const lm_control_struct *control#>, lm_status_struct *status)
    lmmin(1, par, nEqs, &dataStruct, evaluateReprojectionError, &control, &status);
    
    return *par;
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
Mat PoseEstimator::estimatePose(View *v1, View *v2)
{
    return estimatePoseMono(v1, v2);
}

Mat PoseEstimator::estimatePoseMono(View *v1, View *v2)
{
    // 1. assume features are matched
    
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
    
    for(int i = 0 ; i < points1.size(); i++)
    {
        double distToEpipolarLine = projectEpipolarLine(E, points1[i], points2[i]);
        // reject outliers from epipolar geometry
        
    }
    // reject outliers
    rejectOutliers(v1, v2, poseRecoveryStat);
    
    // cout << "recovering pose inliers: " << count << "/" << points1.size()<< endl;
    
    // 4. estimate scale (later)
    Mat poseChange = Mat::eye(4, 4, CV_64F);
    Mat aux = poseChange(Rect(0, 0, 3, 3));
    R.copyTo(aux);
    aux = poseChange(Rect(3, 0, 1, 3));
    t.copyTo(aux);
    v2->setPose(v1->getPose() * poseChange);
    
    
    // 7. return pose
    return poseChange.clone();
}
Mat PoseEstimator::solvePnP(View *v, map<long, Landmark> &landmarkBook)
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
    if(status.rows < 0.3 * v->getLeftFeatureSet().size())
    {
        cout << "insufficient inliers: " << status.rows << "/" << v->getLeftFeatureSet().size() << endl;
        return pose.clone();
    }
    for(int i = 0; i < status.rows; i++)
    {
        newFeatures.push_back(v->getLeftFeatureSet().getFeaturePoints()[i]);
        newIds.push_back(v->getLeftFeatureSet().getIds()[i]);
    }
    cout << "inliers: " << newFeatures.size() << "/" << v->getLeftFeatureSet().size() << endl;
    v->getLeftFeatureSet().setFeaturePoints(newFeatures);
    v->getLeftFeatureSet().setIds(newIds);
    
    return pose.clone();
}
Mat PoseEstimator::estimatePoseMotionOnlyBA(View *v1, View *v2, map<long, Landmark> landmarkBook)
{
    // setting up g2o solver
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver =
    new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
    
    // setting up camer parameters
    double focalLength = CameraParameters::focal;
    Vector2d principalPoint(CameraParameters::principal.x, CameraParameters::principal.y);
    vector<g2o::SE3Quat, aligned_allocator<g2o::SE3Quat> > true_poses;
    g2o::CameraParameters * camParams = new g2o::CameraParameters (focalLength, principalPoint, 0.);
    camParams->setId(0);
    if (!optimizer.addParameter(camParams)) {
        assert(false);
    }
    
    // setting up camera poses as vertices
    vector<g2o::SE3Quat, aligned_allocator<g2o::SE3Quat> > truePoses;
    Mat pose = v1->getPose();
    pose = pose.inv();
    Vector3d t(pose.at<double>(0, 3), pose.at<double>(1, 3), pose.at<double>(2, 3));
    vector<double> quat = rot2quat(pose(Rect(0, 0, 3, 3)));
    Quaterniond q = Quaterniond(quat[0], quat[1], quat[2], quat[3]);
    g2o::SE3Quat g2oPose(q,t);
    g2o::VertexSE3Expmap * v_se3 = new g2o::VertexSE3Expmap();
    v_se3->setId(int(v1->getId()));
    v_se3->setFixed(true);
    v_se3->setEstimate(g2oPose);
    optimizer.addVertex(v_se3);
    
    // v2
    pose = v2->getPose();
    pose = pose.inv();
    t = Vector3d(pose.at<double>(0, 3), pose.at<double>(1, 3), pose.at<double>(2, 3));
    quat = rot2quat(pose(Rect(0, 0, 3, 3)));
    q = Quaterniond(quat[0], quat[1], quat[2], quat[3]);
    g2oPose = g2o::SE3Quat(q,t);
    v_se3 = new g2o::VertexSE3Expmap();
    v_se3->setId(int(v2->getId()));
    v_se3->setEstimate(g2oPose);
    optimizer.addVertex(v_se3);
    
    // setting up landmarks as vertices
    Canvas *canvas = new Canvas();
    const float thHuber = sqrt(5.991);
    for (map<long, Landmark>::iterator landmarkIter = landmarkBook.begin();
         landmarkIter != landmarkBook.end(); landmarkIter++)
    {
        long id = landmarkIter->first;
        Landmark landmark = landmarkIter->second;
        Vector3d point3d = Vector3d(landmark.point3d.x, landmark.point3d.y, landmark.point3d.z);
        g2o::VertexSBAPointXYZ * v_p = new g2o::VertexSBAPointXYZ();
        v_p->setId(int(id));
        v_p->setMarginalized(true);
        v_p->setEstimate(point3d);
        // motion only BA: not updating landmarks
        v_p->setFixed(true);
        optimizer.addVertex(v_p);
        
        // v1
        if(v1->getIdBook().count(id))
        {
            Feature feature = v1->getLeftFeatureSet().getFeatureById(id);
            Vector2d z = Vector2d(feature.getPoint().pt.x, 376 - feature.getPoint().pt.y);
            g2o::EdgeProjectXYZ2UV * e = new g2o::EdgeProjectXYZ2UV();
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>
                         (optimizer.vertices().find(int(v1->getId()))->second));
            e->setMeasurement(z);
            e->information() = Matrix2d::Identity();
            e->setParameterId(0, 0);
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuber);
            optimizer.addEdge(e);

        }
        
        // v2
        if(v2->getIdBook().count(id))
        {
            Feature feature = v2->getLeftFeatureSet().getFeatureById(id);
            Vector2d z = Vector2d(feature.getPoint().pt.x, 376 - feature.getPoint().pt.y);
            g2o::EdgeProjectXYZ2UV * e = new g2o::EdgeProjectXYZ2UV();
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>
                         (optimizer.vertices().find(int(v2->getId()))->second));
            e->setMeasurement(z);
            e->information() = Matrix2d::Identity();
            e->setParameterId(0, 0);
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuber);
            optimizer.addEdge(e);
            
        }
    }
    
    optimizer.setVerbose(false);
    // optimizer.save("/Users/orangechicken/Desktop/SLAM/g2o_data/firstInput.g2o");
    
    // start optimization
    optimizer.initializeOptimization();
    cout << "Performing motion only BA:" << endl;
    optimizer.optimize(10);
    
    // save result
    // optimizer.save("/Users/orangechicken/Desktop/SLAM/g2o_data/firstOutput.g2o");
    cout << "BA Finished" << endl;
    
    // update view2
    long id = v2->getId();
    g2o::HyperGraph::VertexIDMap::iterator it = optimizer.vertices().find(int(id));
        if(it != optimizer.vertices().end())
    {
        g2o::VertexSE3Expmap *v_se3 = dynamic_cast<g2o::VertexSE3Expmap*>(it->second);
        g2o::SE3Quat se3quat = v_se3->estimate();
        Matrix<double, 4, 4> pose = se3quat.to_homogeneous_matrix();
        Mat estimatedPose = Converter::eigenMatToCvMat(pose).inv();
        v2->setPose(estimatedPose);
    }
   
    // return pose change
    Mat poseChange = v1->getPose().inv() * v2->getPose();
    return poseChange.clone();
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
