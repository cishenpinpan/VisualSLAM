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
        
        if(poseRecoveryStat.at<bool>(i, 0) == 0 && abs(distToEpipolarLine) >= 1)
        {
            poseRecoveryStat.at<bool>(i, 0) = 0;
        }
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
    Mat pose = Mat::eye(4, 4, CV_64F);
    Mat aux = pose(Rect(0, 0, 3, 3));
    R.copyTo(aux);
    aux = pose(Rect(3, 0, 1, 3));
    t.copyTo(aux);
    pose = pose.inv();
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
    optimizer.save("/Users/orangechicken/Desktop/SLAM/g2o_data/firstInput.g2o");
    
    // start optimization
    optimizer.initializeOptimization();
    cout << "Performing motion only BA:" << endl;
    optimizer.optimize(10);
    
    // save result
    optimizer.save("/Users/orangechicken/Desktop/SLAM/g2o_data/firstOutput.g2o");
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
