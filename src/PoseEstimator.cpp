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
    
}

double refineScaleForOnlyOnePoint(KeyPoint p1, KeyPoint p2, KeyPoint p3, Mat R12, Mat T12, Mat R21, Mat T21, Mat R23, Mat T23)
{
    Mat gamma2(3,1,CV_64F);
    gamma2.at<double>(0,0) = p2.pt.x;
    gamma2.at<double>(1,0) = p2.pt.y;
    gamma2.at<double>(2,0) = 1;
    
    Mat gamma1(3,1,CV_64F);
    gamma1.at<double>(0,0) = p1.pt.x;
    gamma1.at<double>(1,0) = p1.pt.y;
    gamma1.at<double>(2,0) = 1;
    
    Mat gamma3(3,1,CV_64F);
    gamma3.at<double>(0,0) = p3.pt.x;
    gamma3.at<double>(1,0) = p3.pt.y;
    gamma3.at<double>(2,0) = 1;
    
    
    Mat K = CameraParameters::getIntrinsic();
    gamma2 = K.inv() * gamma2;
    gamma1 = K.inv() * gamma1;
    gamma3 = K.inv() * gamma3;
    
    Mat R23g2 = R23 * gamma2;
    Mat R21g2 = R21 * gamma2;
    
    Mat R12g1 = R12 * gamma1;
    
    double A = T21.at<double>(0,0) - T21.at<double>(2,0) * gamma1.at<double>(0,0);
    double D = R21g2.at<double>(2,0) * gamma1.at<double>(0,0) - R21g2.at<double>(0,0);
    
    double B = T23.at<double>(0,0) - T23.at<double>(2,0) * gamma3.at<double>(0,0);
    double C = R23g2.at<double>(2,0) * gamma3.at<double>(0,0) - R23g2.at<double>(0,0);
    
    double E = R12g1.at<double>(2,0) * T12.at<double>(0,0) - R12g1.at<double>(0,0) * T12.at<double>(2,0);
    double F = R12g1.at<double>(2,0) * gamma2.at<double>(0,0) - R12g1.at<double>(0,0);
    
    if (A/D < 0 || B/C < 0)
        return -1;
    
    //	cout << "rho2_2:" <<  A/D << endl;
    //	cout << "rho2_1:" << E/F << endl;
    //double ratio = (A*C) / (B*D);
    double ratio = (E * C) / (B * F);
    return ratio;
    
}

double evaluateTrinocularReprojectionError(const double *par, const void *data, double *fvec)
{
    double lambda = *par;
    // unpack data
    Trinocular* t = (Trinocular*)data;
    vector<vector<KeyPoint>> keyPoints = t->keyPoints;
    Mat leftPoseToV = t->left.clone();
    Mat rightPoseToV = t->right.clone();
    vector<bool> status = vector<bool>(keyPoints.size(), false);
    
    leftPoseToV.col(3).rowRange(0, 3) = leftPoseToV.col(3).rowRange(0, 3) * lambda;
    rightPoseToV = CameraParameters::getStereoPose().inv() * leftPoseToV;
    const Mat R1 = leftPoseToV.rowRange(0, 3).colRange(0, 3).inv();
    const Mat T1 = -R1 * leftPoseToV.rowRange(0, 3).col(3);
    const Mat R2 = rightPoseToV.rowRange(0, 3).colRange(0, 3).inv();
    const Mat T2 = -R2 * rightPoseToV.rowRange(0, 3).col(3);
    const Mat E1 = Converter::tVecToTx(T1) * R1;
    const Mat E2 = Converter::tVecToTx(T2) * R2;
    const Mat F1 = CameraParameters::getIntrinsic().inv().t() * E1 * CameraParameters::getIntrinsic().inv();
    const Mat F2 = CameraParameters::getIntrinsic().inv().t() * E2 * CameraParameters::getIntrinsic().inv();
    
    double totalError = 0.0;
    int inlierCount = 0, totalCount = 0;
    const double delta = sqrt(REPROJECTION_THRESHOLD * REPROJECTION_THRESHOLD / 2);
    for(int i = 0; i < keyPoints.size(); i++)
    {
        KeyPoint kp1 = keyPoints[i][0], kp2 = keyPoints[i][1], kp = keyPoints[i][2];
        Mat p1(3, 1, CV_64F), p2(3, 1, CV_64F), p(3, 1, CV_64F);
        p1.at<double>(0, 0) = kp1.pt.x;
        p1.at<double>(1, 0) = kp1.pt.y;
        p1.at<double>(2, 0) = 1;
        p2.at<double>(0, 0) = kp2.pt.x;
        p2.at<double>(1, 0) = kp2.pt.y;
        p2.at<double>(2, 0) = 1;
        p.at<double>(0, 0) = kp.pt.x;
        p.at<double>(1, 0) = kp.pt.y;
        p.at<double>(2, 0) = 1;
        const Mat line1 = F1 * p1, line2 = F2 * p2;
        const double a1 = line1.at<double>(0, 0), b1= line1.at<double>(1, 0), c1 = line1.at<double>(2, 0);
        const double a2 = line2.at<double>(0, 0), b2= line2.at<double>(1, 0), c2 = line2.at<double>(2, 0);
        const double x_hat = (b1 * c2 - b2 * c1) / (a1 * b2 - a2 * b1);
        const double y_hat = -(a1 * c2 - a2 * c1) / (a1 * b2 - a2 * b1);
        double err_x = x_hat - kp.pt.x;
        double err_y = y_hat - kp.pt.y;
        
        if(fvec != NULL)
        {
            *(fvec++) = huber(err_x, delta, false);
            *(fvec++) = huber(err_y, delta, false);
        }
        double reprojectionError = sqrt(err_x * err_x + err_y * err_y);
        if(reprojectionError < REPROJECTION_THRESHOLD)
        {
            status[i] = true;
            inlierCount++;
        }
        else
            status[i] = false;
        totalCount++;
        totalError += reprojectionError;
    }
    t->status = vector<bool>(status);
    t->inlierRatio = double(inlierCount) / totalCount;
    return totalError;
}

void evaluateTrinocularReprojectionError(const double *par, const int nEqs, const void *data, double *fvec,int *userBreak)
{
    evaluateTrinocularReprojectionError(par, data, fvec);
}
double PoseEstimator::estimateScaleTrinocularRANSAC(View *stereo, View *v, bool oneDRansac)
{
    const Mat leftPoseToV = estimatePose(stereo, v);
    const Mat rightPoseToV = CameraParameters::getStereoPose().inv() * leftPoseToV;
   
	Mat prevPose = stereo->getPose().clone();
	Mat currPose = v->getPose().clone();

	Mat relativePose = prevPose.inv() * currPose;
	relativePose = relativePose.inv();

	Mat stereoPose = CameraParameters::getStereoPose().clone();
	stereoPose = stereoPose.inv();

	Mat stereoPoseInv = stereoPose.inv();

	Mat R21 = stereoPose.rowRange(0,3).colRange(0,3);
	Mat T21 = stereoPose.rowRange(0,3).colRange(3,4);
	Mat R12 = stereoPoseInv.rowRange(0,3).colRange(0,3);
	Mat T12 = stereoPoseInv.rowRange(0,3).colRange(3,4);
	Mat R23 = relativePose.rowRange(0,3).colRange(0,3);
	Mat T23 = relativePose.rowRange(0,3).colRange(3,4);

    // find common features
    vector<vector<KeyPoint>> keyPoints;
    map<long, int> commonIds;
    FeatureSet featureSet_stereoLeft = stereo->getLeftFeatureSet();
    FeatureSet featureSet_stereoRight = stereo->getRightFeatureSet();
    FeatureSet featureSet_v = v->getLeftFeatureSet();
    for(int i = 0; i < featureSet_stereoLeft.size(); i++)
    {
        long id = featureSet_stereoLeft.getIds()[i];
        if(!commonIds.count(id))
            commonIds[id] = 0;
        commonIds[id]++;
    }
    for(int i = 0; i < featureSet_stereoRight.size(); i++)
    {
        long id = featureSet_stereoRight.getIds()[i];
        if(!commonIds.count(id))
            commonIds[id] = 0;
        commonIds[id]++;
    }
    for(int i = 0; i < featureSet_v.size(); i++)
    {
        long id = featureSet_v.getIds()[i];
        if(!commonIds.count(id))
            commonIds[id] = 0;
        commonIds[id]++;
    }
    // remove non-common feature points
    // record ids
    for(map<long, int>::iterator it = commonIds.begin(); it != commonIds.end(); it++)
    {
        if(it->second == 3)
        {
            keyPoints.push_back({featureSet_stereoLeft.getFeatureById(it->first).getPoint(),
                featureSet_stereoRight.getFeatureById(it->first).getPoint(),
                featureSet_v.getFeatureById(it->first).getPoint()});
        }
    }
    int N = 200;
    int K = 3;
    double maxInlierRatio = 0.0;
    double bestLambda = .99;
    vector<bool> inlierStatus;
    for(int i = 0; i < N; i++)
    {
		double lambda = .99;
		if (oneDRansac)
		{
			KeyPoint oneDBundlePrevR, oneDBundlePrevL, oneDBundleCurr;
			int index = rand() % keyPoints.size();
			oneDBundleCurr = keyPoints[index][2];
			oneDBundlePrevL = keyPoints[index][0];
			oneDBundlePrevR = keyPoints[index][1];
			lambda = refineScaleForOnlyOnePoint(oneDBundlePrevR, oneDBundlePrevL, oneDBundleCurr, R12, T12, R21, T21, R23, T23);
		}
		else
		{
        	// generate k random number between 0 and commonFeatures.size()
        	vector<vector<KeyPoint>> kpsRANSAC;
        	set<int> indices;
        	for(int k = 0; k < K; k++)
        	{
				int index = rand() % keyPoints.size();
            	while(indices.count(index))
            	{
               	 	index = rand() % keyPoints.size();
                	indices.insert(index);
            	}
            	kpsRANSAC.push_back(keyPoints[index]);
        	}
        
        
        	Trinocular ransac(kpsRANSAC, leftPoseToV, rightPoseToV);
        
        	// 2. nonlinear optimization (levenburg marquardt)
        	/* auxiliary parameters */
        	lm_control_struct control = lm_control_double;
        	lm_status_struct  status;
        	control.verbosity = 0;
        
        	
        	double *par = &lambda;
        	const int nEqs = int(kpsRANSAC.size()) * 2;
        	lmmin(1, par, nEqs, &ransac, evaluateTrinocularReprojectionError, &control, &status);
		}
        
        // test this lambda on entire dataset
        Trinocular testData(keyPoints, leftPoseToV, rightPoseToV);
        evaluateTrinocularReprojectionError(&lambda, &testData, NULL);
        // record only the best set of data points and their corresponding ratio
        double inlierRatio = testData.inlierRatio;
        if(maxInlierRatio < inlierRatio)
        {
            maxInlierRatio = inlierRatio;
			//cout << "Final Ratio:" <<maxInlierRatio << endl;
            bestLambda = lambda;
            inlierStatus = vector<bool>(testData.status);
            if(inlierRatio > RANSAC_CONFIDENCE)
                break;
        }
    }
    
    // final rounds
    double lambda = bestLambda;
    // inliers
    vector<vector<KeyPoint>> inlierKps;
    
    for(int i = 0; i < inlierStatus.size(); i++)
    {
        if(inlierStatus[i])
        {
            inlierKps.push_back(keyPoints[i]);
        }
    }
    for(int iter = 0; iter < 1; iter++)
    {
        // optimize this lambda on inlier dataset
        Trinocular testData(inlierKps, leftPoseToV, rightPoseToV);
        const int nEqs = int(inlierKps.size()) * 2;
        // 2. nonlinear optimization (levenburg marquardt)
        /* auxiliary parameters */
        lm_control_struct control = lm_control_double;
        lm_status_struct  status;
        control.verbosity = 0;
        
        double *par = &lambda;
        lmmin(1, par, nEqs, &testData, evaluateTrinocularReprojectionError, &control, &status);
        
        // update inliers
        inlierStatus = vector<bool>(testData.status);
        vector<vector<KeyPoint>> newInlierKps;
        
        for(int i = 0; i < inlierStatus.size(); i++)
        {
            if(inlierStatus[i])
            {
                newInlierKps.push_back(inlierKps[i]);
            }
        }
        inlierKps = vector<vector<KeyPoint>>(newInlierKps);
        
    }
    return lambda;
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
        if(!v1->getLeftFeatureSet().hasId(id))
            continue;
        KeyPoint kp1 = v1->getLeftFeatureSet().getFeatureById(id).getPoint();
        KeyPoint kp2 = v2->getLeftFeatureSet().getFeaturePoints()[i];
//        Canvas canvas;
//        canvas.drawFeatureMatches(v1->getImgs()[0], v2->getImgs()[0], {kp1}, {kp2});
        newKps1.push_back(kp1);
        newKps2.push_back(kp2);
        newIds.push_back(id);
    }
    //    v1->getLeftFeatureSet().setFeaturePoints(newKps1);
    //    v1->getLeftFeatureSet().setIds(newIds);
    //    v2->getLeftFeatureSet().setFeaturePoints(newKps2);
    //    v2->getLeftFeatureSet().setIds(newIds);
    // 2. estimate essential matrix
    Mat essentialMatrixStat;
    
    vector<Point2f> points1, points2;
    KeyPoint::convert(newKps1, points1);
    KeyPoint::convert(newKps2, points2);
    Mat E = findEssentialMat(points2, points1,
                             CameraParameters::getFocal(), CameraParameters::getPrincipal(), RANSAC, 0.999, 1.0, essentialMatrixStat);
    // reject outliers
    // rejectOutliers(v1, v2, essentialMatrixStat);
    
    // cout << "essential matrix inliers: " << count << "/" << points1.size() << endl;
    
    // 3. recorver pose (translation up to a scale)
    Mat poseRecoveryStat;
    Mat R, t;
    KeyPoint::convert(newKps1, points1);
    KeyPoint::convert(newKps2, points2);
    recoverPose(E, points2, points1, R, t,
                CameraParameters::getFocal(), CameraParameters::getPrincipal(), poseRecoveryStat);
    t = lambda * t;
    // reject outliers
    // rejectOutliers(v1, v2, poseRecoveryStat);
    
    // cout << "recovering pose inliers: " << count << "/" << points1.size()<< endl;
    
    // 4. update pose
    Mat poseChange = Mat::eye(4, 4, CV_64F);
    Mat aux = poseChange(Rect(0, 0, 3, 3));
    R.copyTo(aux);
    aux = poseChange(Rect(3, 0, 1, 3));
    t.copyTo(aux);
    // v2->setPose(v1->getPose() * poseChange);
    
    // 7. return pose
    return poseChange.clone();
}
Mat PoseEstimator::solvePnP(View *v, map<long, Landmark> landmarkBook)
{
    vector<Point3d> landmarks;
    vector<KeyPoint> kps;
    vector<Point2f> imagePoints;
    const vector<long> ids = v->getLeftFeatureSet().getIds();
    for(int i = 0; i < v->getLeftFeatureSet().size(); i++)
    {
        KeyPoint kp = v->getLeftFeatureSet().getFeaturePoints()[i];
        long id = v->getLeftFeatureSet().getIds()[i];
        if(!landmarkBook.count(id))
            continue;
        landmarks.push_back(landmarkBook[id].getPoint());
        kps.push_back(kp);
    }
    KeyPoint::convert(kps, imagePoints);
    const Mat cameraMatrix = CameraParameters::getIntrinsic();
    const Mat distCoeffs = CameraParameters::getDistCoeff();
    Mat R, Rvec, t;
    Mat status;
    solvePnPRansac(landmarks, imagePoints,cameraMatrix,distCoeffs, Rvec, t, false, 100, 8.0, 0.9, status, CV_ITERATIVE);
    Rodrigues(Rvec, R);
    Mat pose = Converter::rotationTranslationToPose(R, t);
    pose = pose.inv();
    return pose.clone();
}

double evaluateReprojectionError(const void *data, double *fvec)
{
    // unpack data
    PnP* pnpData = (PnP*)data;
    const Mat currPose = pnpData->pose;
    vector<KeyPoint> keyPoints = pnpData->keyPoints;
    vector<Point3d> p3ds = pnpData->landmarks;
    vector<bool> status = vector<bool>(keyPoints.size(), false);
    
    double totalError = 0.0;
    int inlierCount = 0, totalCount = 0;
    const double delta = sqrt(REPROJECTION_THRESHOLD * REPROJECTION_THRESHOLD / 2);
    for(int i = 0; i < p3ds.size(); i++)
    {
        Point3d p3d = p3ds[i];
        KeyPoint kp = keyPoints[i];
        pair<double, double> err = reproject3DPoint(p3d, currPose, kp, false);
        if(fvec != NULL)
        {
            *(fvec++) = huber(err.first, delta, false);
            *(fvec++) = huber(err.second, delta, false);
        }
        double reprojectionError = sqrt(err.first * err.first + err.second * err.second);
        if(reprojectionError < REPROJECTION_THRESHOLD)
        {
            status[i] = true;
            inlierCount++;
        }
        else
            status[i] = false;
        totalCount++;
        totalError += reprojectionError;
    }
    pnpData->status = vector<bool>(status);
    pnpData->inlierRatio = double(inlierCount) / totalCount;
    return totalError;
}
void evaluateTripletReprojectionError(const double *par, const void *data, double *fvec)
{
    double lambda = *par;
    
    // unpack
    Triplet *dataStruct = (Triplet*) data;
    vector<KeyPoint> keyPoints = dataStruct->keyPoints;
    vector<Point3d> landmarks = dataStruct->landmarks;
    View *v = dataStruct->v;
    const Mat prevPose = dataStruct->prevPose;
    // update pose
    Mat relativePose = prevPose.inv() * v->getPose();
    // multiply by lambda
    relativePose.col(3).rowRange(0, 3) = lambda * relativePose.col(3).rowRange(0, 3);
    const Mat currPose = prevPose * relativePose;
    
    // construct pnp struct
    PnP *pnp = new PnP(currPose, keyPoints, landmarks);
    
    evaluateReprojectionError(pnp, fvec);
    
    // inlier status
    vector<bool> status(landmarks.size(), false);
    for(int i = 0; i < landmarks.size(); i++)
    {
        if(pnp->status[i])
            status[i] = true;
    }
    dataStruct->status = vector<bool>(status);
    dataStruct->inlierRatio = pnp->inlierRatio;
}
void evaluateTripletReprojectionError(const double *par, const int nEqs, const void *data, double *fvec, int *userBreak)
{
    evaluateTripletReprojectionError(par, data, fvec);
}
double PoseEstimator::solveScalePnP(View *v, const Mat prevPose, map<long, Landmark> landmarkBook, double initial)
{
    // cheat using PnP
    Mat newPose = solvePnP(v, landmarkBook);
    Mat relativePose = prevPose.inv() * newPose;
    double scale = norm(relativePose.col(3).rowRange(0, 3));
    return scale;
    
    // take out existing 2d features and their corresponding 3d landmarks
    vector<KeyPoint> kps;
    vector<Point3d> p3ds;
    for(int i = 0; i < v->getLeftFeatureSet().size(); i++)
    {
        KeyPoint kp = v->getLeftFeatureSet().getFeaturePoints()[i];
        long id = v->getLeftFeatureSet().getIds()[i];
        if(!landmarkBook.count(id))
            continue;
        Point3d p3d = landmarkBook[id].getPoint();
        kps.push_back(kp);
        p3ds.push_back(p3d);
    }
    
    if(kps.size() < 20)
    {
        cout << "No common features(" << kps.size() << ")." << endl;
        return 1.0;
    }
    // lm
    // 1 DoF
    const int nEqs = int(kps.size()) * 2;
    Triplet dataStruct(v, prevPose, kps, p3ds);
    
    // 2. nonlinear optimization (levenburg marquardt)
    /* auxiliary parameters */
    lm_control_struct control = lm_control_double;
    lm_status_struct  status;
    control.verbosity = 0;
    
    double lambda = initial;
    double *par = &lambda;
    lmmin(1, par, nEqs, &dataStruct, evaluateTripletReprojectionError, &control, &status);
    cout << "triplet common features: " << kps.size() << endl;
    return lambda;

}
double PoseEstimator::solveScalePnPRANSAC(View *v, View *prevStereo, map<long, Landmark> landmarkBook, double initial, bool oneDRansac)
{
	const Mat prevPose = prevStereo->getPose();
    // take out existing 2d features and their corresponding 3d landmarks
    vector<KeyPoint> kps;
	vector<KeyPoint> kpsPrevLeft;
	vector<KeyPoint> kpsPrevRight;
    vector<Point3d> p3ds;
    for(int i = 0; i < v->getLeftFeatureSet().size(); i++)
    {
        KeyPoint kp = v->getLeftFeatureSet().getFeaturePoints()[i];
		
        long id = v->getLeftFeatureSet().getIds()[i];
        if(!landmarkBook.count(id) || !prevStereo->getLeftFeatureSet().hasId(id) || !prevStereo->getLeftFeatureSet().hasId(id))
            continue;
        Point3d p3d = landmarkBook[id].getPoint();
		KeyPoint kppL = prevStereo->getLeftFeatureSet().getFeatureById(id).getPoint();
		KeyPoint kppR = prevStereo->getRightFeatureSet().getFeatureById(id).getPoint();
        kps.push_back(kp);
        p3ds.push_back(p3d);
		kpsPrevLeft.push_back(kppL);
		kpsPrevRight.push_back(kppR);	
    }
    
    // RANSAC
    srand(time(NULL));
    double maxInlierRatio = 0.0;
    double bestLambda = 1.0;
    vector<bool> inlierStatus(kps.size(), true);

	
	Mat currPose = v->getPose().clone();
	Mat relativePose = prevPose.inv() * currPose;
	relativePose = relativePose.inv();

	Mat stereoPose = CameraParameters::getStereoPose();
	stereoPose = stereoPose.inv();

	Mat stereoPoseInv = stereoPose.inv();
	Mat R21 = stereoPose.rowRange(0,3).colRange(0,3);
	Mat T21 = stereoPose.rowRange(0,3).colRange(3,4);
	Mat R12 = stereoPoseInv.rowRange(0,3).colRange(0,3);
	Mat T12 = stereoPoseInv.rowRange(0,3).colRange(3,4);
	Mat R23 = relativePose.rowRange(0,3).colRange(0,3);
	Mat T23 = relativePose.rowRange(0,3).colRange(3,4);

	

    int N = 200;
	int K = 3;
 	for(int i = 0; i < N; i++)
 	{

		double lambda = initial;
		//P1P
		if (oneDRansac) 
		{
			KeyPoint oneDBundleCurr, oneDBundlePrevL, oneDBundlePrevR;	
			int index = rand() % kps.size();
			oneDBundleCurr = kps[index];
			oneDBundlePrevL = kpsPrevLeft[index];
			oneDBundlePrevR = kpsPrevRight[index];
			lambda = refineScaleForOnlyOnePoint(oneDBundlePrevR, oneDBundlePrevL, oneDBundleCurr, R12, T12, R21, T21, R23, T23);
		}
		//P3P
		else
		{
  			// generate k random number between 0 and commonFeatures.size()
  			vector<Point3d> p3dsRANSAC;
  			vector<KeyPoint> kpsRANSAC;
  			set<int> indices;
  			for(int k = 0; k < K; k++)
  			{
      			int index = rand() % p3ds.size();
      			while(indices.count(index))
      			{
          			index = rand() % p3ds.size();
          			indices.insert(index);
      			}
       			p3dsRANSAC.push_back(p3ds[index]);
     			kpsRANSAC.push_back(kps[index]);
  			}
  			// lm
  			// 1 DoF
  			const int nEqs = int(kpsRANSAC.size()) * 2;
  			Triplet dataStruct(v, prevPose, kpsRANSAC, p3dsRANSAC);
 
  			// 2. nonlinear optimization (levenburg marquardt)
  			/* auxiliary parameters */
  			lm_control_struct control = lm_control_double;
  			lm_status_struct  status;
  			control.verbosity = 0;
 
  			//double lambda = initial;
  			double *par = &lambda;
  			lmmin(1, par, nEqs, &dataStruct, evaluateTripletReprojectionError, &control, &status);
		}  	 
		// test this lambda on entire dataset
		Triplet testData(v, prevPose, kps, p3ds);
   		evaluateTripletReprojectionError(&lambda, &testData, NULL);
   		// record only the best set of data points and their corresponding ratio
    	double inlierRatio = testData.inlierRatio;
    	if(maxInlierRatio < inlierRatio)
    	{
			maxInlierRatio = inlierRatio;
       		bestLambda = lambda;
      		inlierStatus = vector<bool>(testData.status);
       		if(inlierRatio > RANSAC_CONFIDENCE)
				break;
        }
    }

    // final rounds
    double lambda = bestLambda;
    // inliers
    vector<KeyPoint> inlierKps;
    vector<Point3d> inlierP3ds;
    
    for(int i = 0; i < inlierStatus.size(); i++)
    {
        if(inlierStatus[i])
        {
            inlierKps.push_back(kps[i]);
            inlierP3ds.push_back(p3ds[i]);
        }
    }
    for(int iter = 0; iter < 1; iter++)
    {
        // optimize this lambda on inlier dataset
        Triplet testData(v, prevPose, inlierKps, inlierP3ds);
        const int nEqs = int(inlierKps.size()) * 2;
        // 2. nonlinear optimization (levenburg marquardt)
        /* auxiliary parameters */
        lm_control_struct control = lm_control_double;
        lm_status_struct  status;
        control.verbosity = 0;
        
        double *par = &lambda;
        lmmin(1, par, nEqs, &testData, evaluateTripletReprojectionError, &control, &status);
        
        // update inliers
        inlierStatus = vector<bool>(testData.status);
        vector<KeyPoint> newInlierKps;
        vector<Point3d> newInlierP3ds;
        
        for(int i = 0; i < inlierStatus.size(); i++)
        {
            if(inlierStatus[i])
            {
                newInlierKps.push_back(inlierKps[i]);
                newInlierP3ds.push_back(inlierP3ds[i]);
            }
        }
        inlierKps = vector<KeyPoint>(newInlierKps);
        inlierP3ds = vector<Point3d>(newInlierP3ds);
        
    }
    
    return lambda;
}
void evaluateQuadReprojectionError(const void *data, const double lambda, double *fvec)
{
    double inlierRatio = 0.0;
    double reprojectionError = 0.0;
    // unpack data struct
    Quadruple *dataStruct = (Quadruple*) data;
    View *v1 = dataStruct->v1;
    View *v2 = dataStruct->v2;
    View *v3 = dataStruct->v3;
    View *v4 = dataStruct->v4;
    vector<vector<KeyPoint>> keyPoints;
    vector<Point3d> landmarks;
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
   // compute pnp structs
    vector<KeyPoint> keyPoints_3, keyPoints_4;
    for(int i = 0; i < landmarks.size(); i++)
    {
        keyPoints_3.push_back(keyPoints[i][2]);
        keyPoints_4.push_back(keyPoints[i][3]);
    }
    PnP *pnp_3 = new PnP(pose_v3, keyPoints_3, landmarks);
    PnP *pnp_4 = new PnP(pose_v4, keyPoints_4, landmarks);
    if(fvec != NULL)
    {
        evaluateReprojectionError(pnp_3, fvec);
        evaluateReprojectionError(pnp_4, fvec + landmarks.size() * 2);
    }
    else
    {
        evaluateReprojectionError(pnp_3, NULL);
        evaluateReprojectionError(pnp_4, NULL);
    }
    // inlier status
    vector<bool> status(landmarks.size(), false);
    int inlierCount = 0, totalCount = 0;
    for(int i = 0; i < landmarks.size(); i++)
    {
        if(pnp_3->status[i] && pnp_4->status[i])
        {
            status[i] = true;
            inlierCount++;
        }
        totalCount++;
    }
    dataStruct->status = vector<bool>(status);
    dataStruct->inlierRatio = double(inlierCount) / totalCount;
}
void evaluateQuadReprojectionError(const double *par, const int nEqs, const void *data, double *fvec, int *userBreak)
{
    // unpack initial guess (of the pose)
    double lambda = *par;
    // compute reprojection error
    evaluateQuadReprojectionError(data, lambda, fvec);
}


// function : given three views in order, do the following:
//            1) compute up-to-scale relative poses between each pair
//              (setting first view to origin(0, 0, 0))
//              (used only for triangulation and later computation of optimal ratio)
//            2) construct a world using first two
//            3) store absolute poses (up-to-scale)(relative to first view) and landmarks in view tracker

// input    : View *v1, View *v2, View *v3 -> three views IN ORDER
//            ViewTracker *tripletTracker -> view tracker pointer

// output   : void
ViewTracker* PoseEstimator::constructTriplet(View *v1, View *v2, View *v3)
{
    const Mat pose_12 = estimatePose(v1, v2);
    const Mat pose_23 = estimatePose(v2, v3);
    v1->setPose(Mat::eye(4, 4, CV_64F));
    v2->setPose(v1->getPose() * pose_12);
    v3->setPose(v2->getPose() * pose_23);
    ViewTracker *tripletTracker = new ViewTracker();
    tripletTracker->addView(v1);
    tripletTracker->addView(v2);
    // triangulation
    tripletTracker->computeLandmarks(true);
    tripletTracker->addView(v3);
    return tripletTracker;
}
void PoseEstimator::solveRatioInTriplets(vector<View*> keyViews, vector<View*> allViews)
{
    // optimize N-ples;
//    FeatureTracker *featureTracker = new FeatureTracker();
//    int numViews = int(keyViews.size());
//    vector<double> ratios(numViews - 1, 1.0); // size = numViews - 1
//    vector<double> lambdas(numViews - 1, 1.0);
//    vector<Mat> relativePoses(numViews - 1, Mat::eye(4, 4, CV_64F)); // size = numViews - 1
//    Canvas *canvas = new Canvas();
//    double alpha = lambdas[0];
//    // divide views into triplets
//    for(int i = 0; i < numViews - 2; i++)
//    {
//        cout << "keyframe: " << i + 1 << ", " << i + 2 << ", " << i + 3 << endl;
//        ViewTracker *localViewTracker = constructTriplet(keyViews[i], keyViews[i + 1], keyViews[i + 2]);
//        vector<View*> triplet;
//        triplet.push_back(keyViews[i]);
//        triplet.push_back(keyViews[i + 1]);
//        triplet.push_back(keyViews[i + 2]);
//        double ratio = solveScalePnPRANSAC(triplet[2], triplet[1]->getPose(), localViewTracker->getLandmarkBook());
//        ViewTracker *reverse = constructTriplet(triplet[2], triplet[1], triplet[0]);
//        double reverseRatio = solveScalePnPRANSAC(triplet[0], triplet[1]->getPose(), reverse->getLandmarkBook(),1.0,0);
//        reverseRatio = 1.0 / reverseRatio;
//        double ratioErr = abs(reverseRatio - ratio) / min(ratio, reverseRatio);
//        if(ratioErr > RATIO_ERROR_THRESHOLD)
//        {
//            // replace second view with a nearby backup view
//            int viewIndex = triplet[2]->getTime() - 1;
//            vector<View*> backups;
//            if(viewIndex - 1 > triplet[1]->getTime() - 1)
//                backups.push_back(allViews[viewIndex - 1]);
//            if(i + 3 < numViews && viewIndex + 1 < keyViews[i + 3]->getTime() - 1)
//                backups.push_back(allViews[viewIndex + 1]);
//            bool satisfied = false;
//            View *bestBackup = triplet[2];
//            double leastRatioErr = ratioErr;
//            for(View *backup : backups)
//            {
//                // transfer features onto backup view
//                featureTracker->trackAndMatch({triplet[2], backup});
//                backup->setKeyView();
//                // update (bag, viewtracker) with backup
//                triplet[2] = backup;
//                ViewTracker *localViewTracker = constructTriplet(triplet[0], triplet[1], triplet[2]);
//                // do those stuff again
//                ratio = solveScalePnPRANSAC(triplet[2], triplet[1]->getPose(), localViewTracker->getLandmarkBook(),1.0,0);
//                // satisfication test
//                ViewTracker *reverse = constructTriplet(triplet[2], triplet[1], triplet[0]);
//                double reverseRatio = solveScalePnPRANSAC(triplet[0], triplet[1]->getPose(), reverse->getLandmarkBook());
//                reverseRatio = 1.0 / reverseRatio;
//                ratioErr = abs(reverseRatio - ratio) / min(ratio, reverseRatio);
//                if(ratioErr < RATIO_ERROR_THRESHOLD)
//                {
//                    bestBackup = backup;
//                    satisfied = true;
//                    break;
//                }
//            }
//            if(!satisfied)
//                cout << "still unsatisfied!" << endl;
//            // reset key views
//            keyViews[i + 2] = bestBackup;
//        }
//        // store ratios
//        ratios[i + 1] = ratio;
//        alpha *= ratio;
//        lambdas[i + 1] = alpha;
//    }
//    
//    // estimate up-to-scale relative poses
//    for(int i = 1; i < numViews; i++)
//    {
//        Mat relativePose = estimatePose(keyViews[i - 1], keyViews[i]);
//        // normailze translation vector to unit vector
//        relativePose.col(3).rowRange(0, 3) = relativePose.col(3).rowRange(0, 3)
//                                                / norm(relativePose.col(3).rowRange(0, 3));
//        relativePoses[i - 1] = relativePose.clone();
//    }
//    
//    // update poses
//    for(View *v : allViews)
//    {
//        v->unsetKeyView();
//    }
//    keyViews[0]->setPose(Mat::eye(4, 4, CV_64F));
//    keyViews[0]->setKeyView();
//    
//    alpha = 1.0;
//    for(int i = 1; i < keyViews.size(); i++)
//    {
//        keyViews[i]->setKeyView();
//        Mat prevPose = keyViews[i - 1]->getPose();
//        Mat relativePose = relativePoses[i - 1].clone();
//        double ratio = ratios[i - 1];
//        alpha *= ratio;
//        relativePose.col(3).rowRange(0, 3) = alpha * relativePose.col(3).rowRange(0, 3);
//        Mat currPose = prevPose * relativePose;
//        // update pose
//        keyViews[i]->setPose(currPose.clone());
//    }
//    
//    // BA
//    ViewTracker *BATracker = new ViewTracker();
//    BATracker->addView(keyViews[0]);
//    for(int i = 1; i < keyViews.size(); i++)
//    {
//        BATracker->addView(keyViews[i]);
//        BATracker->computeLandmarks(false);
//        if(BATracker->getKeyViews().size() % 10 == 0)
//            BATracker->bundleAdjust(MOTION_STRUCTURE, GLOBAL_BA);
//    }
//    // BATracker->bundleAdjust(MOTION_STRUCTURE, GLOBAL_BA);
    
}
void PoseEstimator::solvePosesPnPStereo(vector<View*> views)
{
    for(int i = 0; i < views.size() - 1; i++)
    {
        vector<View*> bag;
        bag.push_back(views[i]);
        bag.push_back(views[i + 1]);
        ViewTracker *localViewTracker = new ViewTracker();
        localViewTracker->addView(bag[0]);
        localViewTracker->computeLandmarksStereo();
        localViewTracker->addView(bag[1]);
        
        // solve the scale between 2nd and 3rd frame (adjust later)
        const Mat newPose = solvePnP(bag[1], localViewTracker->getLandmarkBook());
        bag[1]->setPose(newPose.clone());
    }
}
void PoseEstimator::refineScaleStereo(vector<View*> views, bool trinocular, bool oneDRansac)
{
    // optimize N-ples;
    vector<double> scales;
    vector<Mat> relativePoses;
    
    for(int i = 0; i < views.size() - 1; i++)
    {
        vector<View*> bag;
        bag.push_back(views[i]);
        bag.push_back(views[i + 1]);
        bag[0]->setPose(Mat::eye(4, 4, CV_64F));
        const Mat relativePose = estimatePose(bag[0], bag[1]);
        const Mat newPose = bag[0]->getPose() * relativePose;
        bag[1]->setPose(newPose.clone());
        ViewTracker *localViewTracker = new ViewTracker();
        localViewTracker->addView(bag[0]);
        localViewTracker->computeLandmarksStereo();
        localViewTracker->addView(bag[1]);
        
        // solve the scale between 2nd and 3rd frame (adjust later)
        double scale = .99;
        // double scale = solveScalePnPRANSAC(bag[1], bag[0]->getPose(), localViewTracker->getLandmarkBook());
        if(trinocular)
            scale = estimateScaleTrinocularRANSAC(bag[0], bag[1],oneDRansac);
        else
            scale = solveScalePnPRANSAC(bag[1], bag[0], localViewTracker->getLandmarkBook(), 1.0 ,oneDRansac);
        cout << "stereo scale(" << i + 1 << "): " << scale << endl;
        
        scales.push_back(scale);
        relativePoses.push_back(relativePose.clone());
    }
    
    // update poses
    for(int i = 1; i < views.size(); i++)
    {
        Mat prevPose = views[i - 1]->getPose();
        Mat relativePose = relativePoses[i - 1].clone();
        double scale = scales[i - 1];
        relativePose.col(3).rowRange(0, 3) = scale * relativePose.col(3).rowRange(0, 3);
        Mat currPose = prevPose * relativePose;
        // update pose
        views[i]->setPose(currPose.clone());
    }
}

void PoseEstimator::refineScaleMultipleFramesWithDistribution(vector<View*> views, int N = 3)
{
    vector<double> scales = {2.07632};
    
    Canvas *canvas;
    vector<Mat> relativePoses;
    relativePoses.push_back(views[0]->getPose().inv() * views[1]->getPose());
    for (int i = 0; i < views.size() - (N-1); i++)
    {
        vector<View*> bundle;
        for (int j = 0; j < N; j++)
        {
            bundle.push_back(views[i+j]);
            
        }
        const Mat relativePose = bundle[1]->getPose().inv() * bundle[2]->getPose();
        //		cout << bundle.size() << endl;
        //		cout << "view1" << endl;
        //		canvas->drawSingleImage(bundle[0]->getImgs()[0]);
        //		cout << "view2" << endl;
        //		canvas->drawSingleImage(bundle[1]->getImgs()[0]);
        //		cout << "view3" << endl;
        //		canvas->drawSingleImage(bundle[2]->getImgs()[0]);
        double scaleRatio = refineScaleForThreeFrames(bundle, i);
        
        
        //		if (checkValue > 0.08)
        //			scaleRatio = cheatScales[i];
        
        relativePoses.push_back(relativePose.clone());
        scales.push_back(scales.back() * scaleRatio);
        cout << scaleRatio << endl;
    }
    
    
    
    for(int i = 1; i < views.size() - (N - 3); i++)
    {
        Mat prevPose = views[i - 1]->getPose();
        Mat relativePose = relativePoses[i - 1].clone();
        double scale = scales[i - 1];
        relativePose.col(3).rowRange(0, 3) = scale * relativePose.col(3).rowRange(0, 3);
        Mat currPose = prevPose * relativePose;
        // update pose
        views[i]->setPose(currPose.clone());
    }
    
    // print lambdas
    //    for(double lambda : scales)
    ///    {
    //       cout << lambda << endl;
    //  }
    
}



double PoseEstimator::refineScaleForThreeFrames(vector<View*> views, int i)
{
    //	cout << "Refine one bundle..." << endl;
    
    ViewTracker viewTracker1;
    viewTracker1.addView(views[0]);
    viewTracker1.addView(views[1]);
    viewTracker1.computeLandmarks(1);
    
    map<long, Landmark> landMarkForBundle1 = viewTracker1.getLandmarkBook();
    //	for (int i = 0; i < commonIds.size(); i++)
    //	{
    //		cout << landMarkForBundle[commonIds[i]].getPoint().z << endl;
    //	}
    ViewTracker viewTracker2;
    viewTracker2.addView(views[1]);
    viewTracker2.addView(views[2]);
    viewTracker2.computeLandmarks(1);
    
    
    ViewTracker viewTracker3;
    viewTracker3.addView(views[1]);
    viewTracker3.addView(views[0]);
    viewTracker3.computeLandmarks(1);
    
    map<long, Landmark> landMarkForBundle2 = viewTracker2.getLandmarkBook();
    map<long, Landmark> landMarkForBundle3 = viewTracker3.getLandmarkBook();
    //Get Common Features
    FeatureSet featureSet1 = views[0]->getLeftFeatureSet();
    FeatureSet featureSet2 = views[1]->getLeftFeatureSet();
    FeatureSet featureSet3 = views[2]->getLeftFeatureSet();
    
    vector<KeyPoint> referenceKeyPoints = featureSet1.getFeaturePoints();
    vector<long> iDs = featureSet1.getIds();
    vector<KeyPoint> commonKeyPoints1, commonKeyPoints2, commonKeyPoints3;
    vector<long> commonIds;
    for (int i = 0; i < referenceKeyPoints.size(); i++)
    {
        long iD = iDs[i];
        if (!featureSet2.hasId(iD) || !featureSet3.hasId(iD))
            continue;
        
        Feature featureFrom2 = featureSet2.getFeatureById(iD);
        Feature featureFrom3 = featureSet3.getFeatureById(iD);
        
        commonKeyPoints1.push_back(referenceKeyPoints[i]);
        commonKeyPoints2.push_back(featureFrom2.getPoint());
        commonKeyPoints3.push_back(featureFrom3.getPoint());
        commonIds.push_back(iD);
    }
    
    Canvas canvas;
    canvas.drawTrackingPathEveryOtherK(views[0]->getImgs()[0], commonKeyPoints1, commonKeyPoints2, 1);
    //	canvas.drawTrackingPathEveryOtherK(views[1]->getImgs()[0], commonKeyPoints2, commonKeyPoints3, 1);
    
    //Calculate scale for all set triplet
    vector<double> scales;
    vector<double> scalesFromClose;
    Mat pose1 = views[0]->getPose();
    Mat pose2 = views[1]->getPose();
    Mat pose3 = views[2]->getPose();
    
    Mat relativePoseCamera12 = pose1.inv() * pose2;
    Mat relativePoseCamera23 = pose2.inv() * pose3;
    
    Mat relativePosePoint21 = relativePoseCamera12;
    Mat relativePosePoint23 = relativePoseCamera23.inv();
    
    Mat relativePose12 = pose1.inv() * pose2;
    Mat relativePose21 = relativePose12.inv();
    
    //Mat relativePose21 = relativePose12;
    Mat R12 = relativePose21.rowRange(0,3).colRange(0,3);
    Mat T12 = relativePose21.rowRange(0,3).colRange(3,4);
    Mat R21 = relativePosePoint21.rowRange(0,3).colRange(0,3);
    Mat T21 = relativePosePoint21.rowRange(0,3).colRange(3,4);
    Mat R23 = relativePosePoint23.rowRange(0,3).colRange(0,3);
    Mat T23 = relativePosePoint23.rowRange(0,3).colRange(3,4);
    //	cout <<"R12" <<R12 << endl;
    //	cout << "R21" << R21 << endl;
    //	cout << "T12" << T12 << endl;
    //	cout << "T21" << T21 << endl;
    for (int i = 0; i < commonIds.size();i++)
    {
        //	double scaleCandidate = refineScaleForSinglePoint(depth,views[0]->getPose(), views[1]->getPose(), views[2]->getPose());
        
        
        
        
        double z2 = landMarkForBundle2[commonIds[i]].getPoint().z;
        if (z2 == 0)
            continue;
        Point3d Gamma1 = landMarkForBundle1[commonIds[i]].getPoint();
        
        if (Gamma1.z == 0)
            continue;
        double z2_21 = landMarkForBundle3[commonIds[i]].getPoint().z;
        if (z2_21 == 0)
            continue;
        
        Mat Gamma1Bar(3,1,CV_64F);
        Gamma1Bar.at<double>(0,0) = Gamma1.x;
        Gamma1Bar.at<double>(1,0) =	Gamma1.y;
        Gamma1Bar.at<double>(2,0) = Gamma1.z;
        
        Mat Gamma2Bar = R12 * Gamma1Bar + T12;
        //		cout <<"z1bar:" << Gamma1Bar.at<double>(2,0) << endl;
        //		cout <<"z2bar:" << Gamma2Bar.at<double>(2,0) << endl;
        //		cout << "z2_21" << z2_21 << endl;
        //		cout << "z2: " << z2 << endl;
        //		cout << endl;
        //		double scaleCandidate =  Gamma2Bar.at<double>(2,0) / z2_21;
        
        double scaleCandidate = z2_21 / z2;
        double ratioFromClose = refineScaleForOnlyOnePoint(commonKeyPoints1[i], commonKeyPoints2[i], commonKeyPoints3[i], R12, T12, R21, T21, R23, T23);
        
        //	cout << scaleCandidate << "," << ratioFromClose << endl;
        scales.push_back(scaleCandidate);
        scalesFromClose.push_back(ratioFromClose);
    }
    vector<double> scalesFinal;
    vector<double> scalesFromCloseFinal;
    for(int i = 0; i < scales.size(); i++)
    {
        if (scales[i] > 0)
            scalesFinal.push_back(scales[i]);
        
        if (scalesFromClose[i] > 0)
            scalesFromCloseFinal.push_back(scalesFromClose[i]);
        
    }
    
    //	ofstream out;
    //	stringstream A;
    //	A << FrameCounter;
    //	string count;
    //	A >> count;
    //	out.open(OUTPUT_DIR + count + "_distribution.output");
    //	for (int i = 0; i < scalesFromCloseFinal.size(); i++)
    //	{
    //		out << scalesFromCloseFinal[i] << endl;
    //	}
    // FrameCounter++;
    
    double medClose = median(scalesFromCloseFinal);
    
    double med = median(scalesFinal);
    double mea = mean(scalesFinal);
    
    double med150 = 4;
    double med50 = 0.05;
    
    vector<double> scalesAfterRejection;
    for (int i = 0; i < scalesFinal.size(); i++)
    {
        if(scalesFromCloseFinal[i] < med150 && scalesFromCloseFinal[i] > med50)
            scalesAfterRejection.push_back(scalesFromCloseFinal[i]);
    }
    
    double var = standardDeviation(scalesFromCloseFinal);
    double medRejection = median(scalesAfterRejection);
    //	cout << var << endl;
    //	cout << " cheat" << cheatScales[i] << endl;
    //	cout << "med Close:" << medClose << "standardDeviation:" << var <<" Med AfterRejection" <<median(scalesAfterRejection) << endl;
    ///	for (int i = 0; i < scalesFinal.size(); i++)
    ///	{
    ///		cout << scalesFinal[i] << endl;
    ///	}
    return medRejection;
}
