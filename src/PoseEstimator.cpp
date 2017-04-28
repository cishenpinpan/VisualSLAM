//
//  PoseEstimator.cpp
//  VisualSLAM
//
//  Created by Rong Yuan on 3/4/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#include "PoseEstimator.h"

vector<double> cheatScales = {1.666241,0.800249,0.748682,1.711935,0.828371,1.025557,1.275837,1.026220,1.480863,1.103866,0.729757,0.976683,1.084049,1.438873,0.670365,0.582779,0.450772,1.448474,0.662443,1.027766,1.331040,1.154723,1.405345,1.407354,1.333145,0.912764,0.709861,0.984781,0.975777,1.506748,1.563540,0.217583,0.232662,3.027510,0.658222,2.034488,1.799758,1.812118,1.227126,0.950643,1.318482,0.811185,1.280838,0.576791,0.570822,1.520571,1.510023,1.228051,0.743973,1.008386,0.614489,0.596107,0.978324,0.864355,2.333449,1.896180,0.640915,0.782261,0.871269,0.974426,1.914865,0.528815,0.212948,1.929174,1.830854,1.004487,1.593402,0.581100,1.115711,2.176555,1.014946,0.628328,1.170360,1.288439,0.487221,0.976390,1.402194,1.860286,0.767178,0.500938,0.767117,0.839335,0.951688,1.455409,1.654124,1.170944,1.296370,0.540124,1.584478,0.864527,0.808824,1.501817,0.817148,1.188460,1.163970,0.724989,0.815404,1.530691,1.156425,0.839259,0.950604,1.228683,1.272580,0.739569,0.569657,0.371232,1.033340,1.066163,1.438987,0.796792,2.136555,0.728898,1.369799,1.099692,0.623010,1.360085,0.750018,1.323362,1.244691,0.789559,1.243554,0.794218,0.985690,0.992094,2.091680,0.782627,0.822363,0.789543,0.908639,0.864974,1.099440,2.781630,0.776676,0.580868,0.990909,0.989941,0.990680,0.990967,0.993058,1.232982,1.366406,0.928039,0.623520,1.173466,0.991301,0.825388,0.433559,1.380674,0.803948,1.794476,1.068898,1.095241,1.551975,0.778018,0.840929,1.525656,1.002981,0.673819,1.264209,1.395414,0.565178,1.233604,0.596743,1.991309,0.847334,0.611083,1.343826,1.270667,1.202904,1.006931,0.839503,1.007812,0.788033,0.987285,0.974707,0.715064,1.264676,1.393109,0.723542,0.845351,1.266086,0.813183,0.445541,1.076464,1.496923,1.730704,1.324240,1.083686,1.081012,0.917896,0.677163,1.498367,0.992709,1.108572,0.521193,1.291678,0.454738,1.647977,1.352162,0.974968,0.421594,0.980059,0.655466,2.526757,2.331233,1.117516,0.163606,0.945491,1.265554,1.985191,1.461934,0.786565,1.130889,1.098075,1.257344,1.052947,1.033520,0.668610,1.680027,0.695037,1.919005,0.874569,0.702109,1.365090,0.532847,0.454239,0.960345,1.465649,2.068871,1.464729,1.074825,0.550572,1.050662,0.771725,2.049678,0.680313,1.015148,1.019203,1.018805,1.279704,1.226678,0.506395,1.341078,1.262156,0.797957,0.996974,1.235668,0.784637,1.189462,1.619219,0.736547,0.959298,0.490023,0.481250,0.974469,1.683914,1.187277,0.858059,0.863196,1.642733,1.552866,1.138682,0.753919,0.656649,1.239707,1.225866,2.091932,0.652131,0.630636,1.635318,0.912151,0.750542,1.198467,0.819501,1.168487,1.129249,0.889777,0.965089,0.732285,1.056453,0.887208,0.841348,1.062086,1.594375,1.068093,0.671070,1.251893,0.792319,0.988378,0.742517,1.653920,0.765157,1.142440,0.700039,0.670250,1.858097,1.124085,0.600060,0.786161,1.745906,1.701633,0.659223,0.826426,0.758729,1.668804,1.233532,1.578037,0.561532,0.606423,1.321355,1.277205,1.435843,0.724897,1.005432,0.989090,0.985988,1.556597,0.355398,0.959168,1.238280,0.874505,1.127334,0.799029,1.592464,0.726141,0.388521,1.255998,1.427626,1.020177,0.983927,1.463410,1.533251,0.724982,1.684077,0.725101,1.030066,0.780582,2.379166,0.729036,0.597154,2.356136,0.430072,0.993816,1.673410,0.995161,0.595955,0.994976,1.669021,0.591014,0.675873,1.979918,1.002545,0.986736,0.740632,0.995501,0.989891,0.981611,2.237742,1.149431,0.793931,1.114399,0.332031,1.389332,0.781387,0.976210,1.045884,1.528042,1.527067,1.297715,0.665773,1.220570,0.991195,1.350988,0.762820,1.331583,0.372410,1.319634,0.984131,0.982904,1.676344,1.363435,0.559356,1.187674,0.918568,0.913346,1.098675,1.309616,1.114276,0.567836,1.372982,0.660474,1.029893,0.999295,0.685735,1.028342,1.326814,1.138968,1.540939,0.697308,1.062799,0.816489,0.754759,1.324701,1.229344,0.974498,0.400587,1.965136,0.494951,1.485558,1.286103,0.706659,1.236207,1.673517,0.549325,0.971487,0.795130,1.011615,1.328084,1.760806,0.547155,2.378294,1.375177,1.130007,0.891058,0.445940,0.993196,1.241108,0.999816,1.190507,0.982887,1.150523,0.847479,1.493088,0.785383,1.294021,1.090816,0.986691,0.942445,0.657206,0.615839,1.096933,1.776241,0.680509,0.850014,1.525037,1.484613,0.762192,0.823960,0.618436,1.594973,1.182188,0.666994,0.361992,0.714107,1.273135,1.046517,1.780579,1.890309,1.541070,0.734995,0.761551,0.823019,1.014142,1.755712,1.131129,0.737524,0.647642,0.988518,0.989393,0.983268,1.483674,0.659918,0.989874,1.717066,0.986281,0.844095,0.980907,0.812715,1.776797,0.526753,1.036164,0.655835,0.799935,1.162186,1.261168,1.226328,0.922632,1.399094,0.646251,1.016604,1.336511,1.745169,0.990630,1.126386,0.744442,0.689211,1.031646,0.750187,1.333135,1.499744,0.825226,1.552633,1.056054,0.203552,1.219346,1.073142,1.340707,0.636613,1.446532,1.368619,1.835769,0.701639,1.072550,0.825968,1.015630,1.267090,1.000231,0.398753,1.485709,1.320569,0.985033,0.986474,0.740687,1.316754,2.331570,0.589799,1.028195,0.595806,0.412810,1.332034,1.094480,1.084813,1.074877,1.692381,2.001486,0.702744,0.524406,1.709569,0.811866,0.997623,0.999846,1.250523,0.403719,2.044699,2.328864,0.688667,1.340571,0.493851,0.993320,1.491530,0.659650,0.735875,1.336749,1.452411,0.492645,0.961574,0.641538,1.457935,1.529507,0.899368,0.529699,0.905736,0.896703,1.178610,0.449171,0.942428,1.483614,1.971636,0.700457,1.382293,0.904990,2.581933,0.674959,1.357151,0.928619,0.621583,1.011239,1.332909,0.984647,1.235100,0.789205,0.494736,1.485654,0.987088,1.258509,0.898100,1.223923,0.991985,0.820948,0.715055,1.087850,1.299158,1.447123,0.580224,2.972483,0.359674,0.332979,2.495678,1.498533,0.462990,3.166619,0.745361,1.117403,1.323481,1.100084,0.683781,0.995793,1.240161,0.782757,0.964371,1.373078,1.313833,0.836729,0.475231,0.816509,0.778789,1.193036,1.462991,1.981347,1.383894,0.501604,1.326409,1.041122,0.809125,1.261929,1.208909,1.007701,1.008935,0.664019,1.246284,0.987332,1.171585,0.777773,1.289504,1.193621,0.888272,0.446655,0.578353,0.766010,1.377660,0.539801,1.534164,1.458925,1.402713,1.136001,1.105687,0.866249,0.784057,1.034883,1.358746,1.264602,0.799216,0.991531,0.744895,1.312646,1.211666,1.944192,0.258504,1.689493,0.834477,1.063966,1.066265,2.166214,0.662629,0.798044,0.741727,1.329013,0.744914,1.318915,0.750787,1.313198,1.477542,0.812018,0.788257,1.503530,0.671847,0.507517,1.502682,2.032068,1.020776,0.500122,1.652190,0.590818,1.587846,0.747337,1.156350,1.041672,1.404954,0.474662,1.018330,0.912076,2.391066,0.411151,0.795640,0.791044,0.849038,1.102791,1.104565,1.100325,2.661929,0.487057,1.824908,0.649865,1.390832,1.306637,0.823986,0.999519,1.237513,0.594503,1.321930,0.495413,1.988564,1.245544,0.803370,1.008766,0.504019};

PoseEstimator::PoseEstimator()
{
    
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
double PoseEstimator::solveScalePnPRANSAC(View *v, const Mat prevPose, map<long, Landmark> landmarkBook, double initial)
{
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
    
    // RANSAC
    srand(time(NULL));
    double maxInlierRatio = 0.0;
    double bestLambda = 1.0;
    vector<bool> inlierStatus(kps.size(), true);
    int N = 200;
    int K = 3;
    for(int i = 0; i < N; i++)
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
        
        double lambda = initial;
        double *par = &lambda;
        lmmin(1, par, nEqs, &dataStruct, evaluateTripletReprojectionError, &control, &status);
        
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
double PoseEstimator::refineScaleRANSAC(vector<View*> views, map<long, Landmark> &landmarkBook)
{
    // unpack 4 frames
    View *v1 = views[0], *v2 = views[1], *v3 = views[2], *v4 = views[3];
    // find common features in four frames
    vector<vector<KeyPoint>> keyPoints;
    vector<Point3d> landmarks;
    multiset<long> commonIds;
    FeatureSet featureSet_v1 = v1->getLeftFeatureSet();
    FeatureSet featureSet_v2 = v2->getLeftFeatureSet();
    FeatureSet featureSet_v3 = v3->getLeftFeatureSet();
    FeatureSet featureSet_v4 = v4->getLeftFeatureSet();
    for(int i = 0; i < featureSet_v1.size(); i++)
    {
        long id = featureSet_v1.getIds()[i];
        commonIds.insert(id);
    }
    for(int i = 0; i < featureSet_v2.size(); i++)
    {
        long id = featureSet_v2.getIds()[i];
        commonIds.insert(id);
    }
    for(int i = 0; i < featureSet_v3.size(); i++)
    {
        long id = featureSet_v3.getIds()[i];
        commonIds.insert(id);
    }
    for(int i = 0; i < featureSet_v4.size(); i++)
    {
        long id = featureSet_v4.getIds()[i];
        commonIds.insert(id);
    }
    // remove non-common feature points
    // record ids
    for(multiset<long>::iterator it = commonIds.begin(); it != commonIds.end(); it++)
    {
        if(commonIds.count(*it) == 4 && landmarkBook.count(*it))
        {
            landmarks.push_back(landmarkBook[*it].getPoint());
            keyPoints.push_back({featureSet_v1.getFeatureById(*it).getPoint(),
                                 featureSet_v2.getFeatureById(*it).getPoint(),
                                 featureSet_v3.getFeatureById(*it).getPoint(),
                                 featureSet_v4.getFeatureById(*it).getPoint()});
        }
    }
    if(landmarks.size() < 20)
        return 1.0;
    // RANSAC
    srand(time(NULL));
    double maxInlierRatio = 0.0;
    double bestLambda = 1.0;
    vector<bool> inlierStatus;
    int N = 100;
    int K = 3;
    for(int i = 0; i < N; i++)
    {
        // generate k random number between 0 and commonFeatures.size()
        vector<Point3d> landmarksRANSAC;
        vector<vector<KeyPoint>> keyPointsRANSAC;
        set<long> indices;
        for(int k = 0; k < K; k++)
        {
            long randomIndex = rand() % landmarks.size();
            while(indices.count(randomIndex))
                randomIndex = rand() % landmarks.size();
            landmarksRANSAC.push_back(landmarks[randomIndex]);
            keyPointsRANSAC.push_back(keyPoints[randomIndex]);
        }
        // lm
        // 1 DoF
        double lambda = .99;
        
        double *par = &lambda;
        
        Quadruple *quad =
                new Quadruple(v1, v2, v3, v4, keyPointsRANSAC, landmarksRANSAC);
        const int nEqs = int(landmarks.size()) * 2 * 2;
        // 2. nonlinear optimization (levenburg marquardt)
        /* auxiliary parameters */
        lm_control_struct control = lm_control_double;
        lm_status_struct  status;
        
        control.verbosity = 0;
        
        lmmin(1, par, nEqs, quad, evaluateQuadReprojectionError, &control, &status);
        
        
        // test this lambda on entire dataset
        Quadruple *completeDataStruct =
                        new Quadruple(v1, v2, v3, v4, keyPoints, landmarks);
        evaluateQuadReprojectionError(completeDataStruct, lambda, NULL);
        // record only the best set of data points and their corresponding ratio
        double inlierRatio = completeDataStruct->inlierRatio;
        if(maxInlierRatio < inlierRatio)
        {
            maxInlierRatio = inlierRatio;
            bestLambda = lambda;
            inlierStatus = vector<bool>(completeDataStruct->status);
        }
    }
    
    
    // final test
    // take out inliers
    vector<Point3d> inlierLandmarks;
    vector<vector<KeyPoint>> inlierKeyPoints;
    
    for(int i = 0; i < inlierStatus.size(); i++)
    {
        if(inlierStatus[i])
        {
            inlierLandmarks.push_back(landmarks[i]);
            inlierKeyPoints.push_back(keyPoints[i]);
        }
    }
    double lambda = bestLambda;
    
    double *par = &lambda;
    Quadruple *quad =
                    new Quadruple(v1, v2, v3, v4, inlierKeyPoints, inlierLandmarks);
    const int nEqs = int(inlierLandmarks.size()) * 2 * 2;
    // 2. nonlinear optimization (levenburg marquardt)
    /* auxiliary parameters */
    lm_control_struct control = lm_control_double;
    lm_status_struct  status;
    
    control.verbosity = 0;
    
    lmmin(1, par, nEqs, quad, evaluateQuadReprojectionError, &control, &status);
    
    cout << "quadruple lambda(set of " << inlierLandmarks.size() << "): " << lambda << endl;
    cout << "final inliers: " << quad->inlierRatio * 100 << "%" << endl;
    
    return lambda;
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
    FeatureTracker *featureTracker = new FeatureTracker();
    int numViews = int(keyViews.size());
    vector<double> ratios(numViews - 1, 1.0); // size = numViews - 1
    vector<Mat> relativePoses(numViews - 1, Mat::eye(4, 4, CV_64F)); // size = numViews - 1
    Canvas *canvas = new Canvas();
    // divide views into triplets
    for(int i = 0; i < numViews - 2; i++)
    {
        ViewTracker *localViewTracker = constructTriplet(keyViews[i], keyViews[i + 1], keyViews[i + 2]);
        vector<View*> triplet;
        triplet.push_back(keyViews[i]);
        triplet.push_back(keyViews[i + 1]);
        triplet.push_back(keyViews[i + 2]);
        double ratio = solveScalePnPRANSAC(triplet[2], triplet[1]->getPose(), localViewTracker->getLandmarkBook());
        // motion model
        MotionModel *model = new MotionModel();
        double ratioFromMotionModel = model->getDistanceRatio(triplet[0], triplet[1], triplet[2]);
        
        cout << "ratio: " << ratio << endl;
        ViewTracker *reverse = constructTriplet(triplet[2], triplet[1], triplet[0]);
        double reverseRatio = solveScalePnPRANSAC(triplet[0], triplet[1]->getPose(), reverse->getLandmarkBook());
        if(abs(1.0 / reverseRatio - ratio) / ratio > CHEAT_THRESHOLD)
        {
            cout << "ratio: " << ratio << endl;
            cout << "reversedRatio: " << 1 / reverseRatio << endl;
            cout << "motion model: " << ratioFromMotionModel << endl;
            // replace second view with a nearby backup view
            int viewIndex = triplet[2]->getTime() - 1;
            vector<View*> backups;
            backups.push_back(allViews[viewIndex - 1]);
            backups.push_back(allViews[viewIndex + 1]);
            // for each backup in order
            bool satisfied = false;
            View *bestBackup = triplet[2];
            for(View *backup : backups)
            {
                // transfer features onto backup view
                featureTracker->trackAndMatch({triplet[2], backup});
                backup->setKeyView();
                // update (bag, viewtracker) with backup
                triplet[2] = backup;
                ViewTracker *localViewTracker = constructTriplet(triplet[0], triplet[1], triplet[2]);
                // do those stuff again
                cout << "Back up! Replacing " << ratio << " with ";
                ratio = solveScalePnPRANSAC(triplet[2], triplet[1]->getPose(), localViewTracker->getLandmarkBook());
                cout << ratio << endl;
                // satisfication test
                ViewTracker *reverse = constructTriplet(triplet[2], triplet[1], triplet[0]);
                double reverseRatio = solveScalePnPRANSAC(triplet[0], triplet[1]->getPose(), reverse->getLandmarkBook());
                if(abs(1.0 / reverseRatio - ratio) / ratio < CHEAT_THRESHOLD)
                {
                    bestBackup = backup;
                    satisfied = true;
                    break;
                }
            }
            if(!satisfied)
                cout << "still unsatisfied!" << endl;
            // update "keyViews", "relative poses", and "ratios" after finding a best backup
            // 1. key views
            keyViews[i + 2] = bestBackup;
//            // 2. relative poses ((i, i+1), (i+1, i+2))
//            const Mat pose_01 = relativePoses[i - 1];
//            const Mat pose_12 = estimatePose(keyViews[i], keyViews[i + 1]);
//            const Mat pose_23 = estimatePose(keyViews[i + 1], keyViews[i + 2]);
//            relativePoses[i] = pose_12.clone();
//            relativePoses[i + 1] = pose_23.clone();
            // 3. ratio in previous triplet -> (i-1, i, i+1)
//            localViewTracker = constructTriplet(keyViews[i - 1], keyViews[i], keyViews[i + 1]);
//            const double ratio_012 = solveScalePnPRANSAC(keyViews[i + 1],
//                                                         keyViews[i]->getPose(), localViewTracker->getLandmarkBook());
//            ratios[i] = ratio_012;
        }
        // store ratios
        ratios[i + 1] = ratio;
        
        // cheat
        double estimateVsMotionModel = abs(ratio - ratioFromMotionModel) / ratioFromMotionModel;
        
    }
    
    // estimate up-to-scale relative poses
    for(int i = 1; i < numViews; i++)
    {
        Mat relativePose = estimatePose(keyViews[i - 1], keyViews[i]);
        // normailze translation vector to unit vector
        relativePose.col(3).rowRange(0, 3) = relativePose.col(3).rowRange(0, 3)
        / norm(relativePose.col(3).rowRange(0, 3));
        relativePoses[i - 1] = relativePose.clone();
    }
    
    // update poses
    double alpha = 1.0;
    keyViews[0]->setPose(Mat::eye(4, 4, CV_64F));
    for(int i = 1; i < keyViews.size(); i++)
    {
        Mat prevPose = keyViews[i - 1]->getPose();
        Mat relativePose = relativePoses[i - 1].clone();
        double ratio = ratios[i - 1];
        alpha *= ratio;
        relativePose.col(3).rowRange(0, 3) = alpha * relativePose.col(3).rowRange(0, 3);
        Mat currPose = prevPose * relativePose;
        // update pose
        keyViews[i]->setPose(currPose.clone());
    }
    
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
void PoseEstimator::refineScaleStereo(vector<View*> views)
{
    // optimize N-ples;
    vector<double> scales;
    vector<Mat> relativePoses;
    for(int i = 0; i < views.size() - 1; i++)
    {
        vector<View*> bag;
        bag.push_back(views[i]);
        bag.push_back(views[i + 1]);
        const Mat relativePose = bag[0]->getPose().inv() * bag[1]->getPose();
        ViewTracker *localViewTracker = new ViewTracker();
        localViewTracker->addView(bag[0]);
        FeatureTracker *featureTracker = new FeatureTracker();
        localViewTracker->computeLandmarksStereo();
        localViewTracker->addView(bag[1]);
        
        // solve the scale between 2nd and 3rd frame (adjust later)
        double scale = solveScalePnPRANSAC(bag[1], bag[0]->getPose(), localViewTracker->getLandmarkBook());
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
    
    // print lambdas
    for(double lambda : scales)
    {
        cout << lambda << endl;
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
        
        double checkValue = abs(scaleRatio - cheatScales[i]) / cheatScales[i];
        
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
    //	double ratio = (A*C) / (B*D);
    double ratio = (E * C) / (B * F);
    return ratio;
    
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
