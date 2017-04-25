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
double evaluateReprojectionError(const double lambda, const void *data, double *fvec)
{
    // unpack data
    ScaleEstimatorStruct *dataStruct = (ScaleEstimatorStruct*) data;
    vector<KeyPoint> keyPoints = dataStruct->keyPoints;
    vector<Point3d> p3ds = dataStruct->landmarks;
    View *v = dataStruct->v;
    const Mat prevPose = dataStruct->prevPose;
    vector<bool> status(p3ds.size(), true);
    
    // update pose
    Mat relativePose = prevPose.inv() * v->getPose();
    // multiply by lambda
    relativePose.col(3).rowRange(0, 3) = lambda * relativePose.col(3).rowRange(0, 3);
    const Mat currPose = prevPose * relativePose;
    
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
   
    dataStruct->status = vector<bool>(status);
    dataStruct->inlierRatio = double(inlierCount) / totalCount;
    return totalError;
}
void evaluateReprojectionError(const double *par, const int nEqs, const void *data, double *fvec, int *userBreak)
{    
    double lambda = *par;
    evaluateReprojectionError(lambda, data, fvec);
}
double solveScaleSinglePoint(View *v1, View* v2, View* v3, vector<KeyPoint> kps, Point3d p3d)
{
    // form several entities first
//    Mat gamma_
    Mat pose_21 = v2->getPose().inv() * v1->getPose();
    Mat pose_23 = v2->getPose().inv() * v3->getPose();
    Mat R_21 = pose_21.rowRange(0, 3).colRange(0, 3).clone();
    Mat R_23 = pose_23.rowRange(0, 3).colRange(0, 3).clone();
    Mat T_21 = pose_21.rowRange(0, 3).col(3).clone();
    Mat T_23 = pose_23.rowRange(0, 3).col(3).clone();
    Mat e_1 = Mat::zeros(3, 1, CV_64F);
    e_1.at<double>(0, 0) = 1;
    Mat e_2 = Mat::zeros(3, 1, CV_64F);
    e_2.at<double>(1, 0) = 1;
    Mat a_1_21 = e_1.t() * R_21;
    Mat a_2_21 = e_2.t() * T_21;
    Mat a_1_23 = e_1.t() * T_23;
    Mat a_2_23 = e_2.t() * T_23;
    Mat b_1_21 = e_1.t() * T_21;
    Mat b_2_21 = e_2.t() * T_21;
    Mat b_1_23 = e_1.t() * T_23;
    Mat b_2_23 = e_2.t() * T_23;
    
    
    return 1.0;
}
double PoseEstimator::solveScaleDist(View *v1, View* v2, View* v3, map<long, Landmark> landmarkBook, double initial)
{
    // solve lambda for each single point
    
    // take out existing 2d features and their corresponding 3d landmarks
    multiset<long> validIds;
    vector<vector<KeyPoint>> kps;
    vector<Point3d> p3ds;
    FeatureSet featureSet_v1 = v1->getLeftFeatureSet();
    FeatureSet featureSet_v2 = v2->getLeftFeatureSet();
    FeatureSet featureSet_v3 = v3->getLeftFeatureSet();
    for(int i = 0; i < featureSet_v1.size(); i++)
    {
        long id = featureSet_v1.getIds()[i];
        validIds.insert(id);
    }
    for(int i = 0; i < featureSet_v2.size(); i++)
    {
        long id = featureSet_v2.getIds()[i];
        validIds.insert(id);
    }
    for(int i = 0; i < featureSet_v3.size(); i++)
    {
        long id = featureSet_v3.getIds()[i];
        validIds.insert(id);
    }
    // remove non-common feature points
    for(set<long>::iterator it = validIds.begin(); it != validIds.end(); it++)
    {
        if(validIds.count(*it) == 3 && landmarkBook.count(*it))
        {
            kps.push_back({v1->getLeftFeatureSet().getFeatureById(*it).getPoint(),
                            v2->getLeftFeatureSet().getFeatureById(*it).getPoint(),
                            v3->getLeftFeatureSet().getFeatureById(*it).getPoint()});
            p3ds.push_back(landmarkBook[*it].getPoint());
        }
    }

    for(int i = 0; i < kps.size(); i++)
    {
        double lambda = solveScaleSinglePoint(v1, v2, v3, kps[i], p3ds[i]);
    }
    return 1.0;
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
    ScaleEstimatorStruct dataStruct(v, prevPose, kps, p3ds);
    
    // 2. nonlinear optimization (levenburg marquardt)
    /* auxiliary parameters */
    lm_control_struct control = lm_control_double;
    lm_status_struct  status;
    control.verbosity = 0;
    
    double lambda = initial;
    double *par = &lambda;
    lmmin(1, par, nEqs, &dataStruct, evaluateReprojectionError, &control, &status);
    cout << "triplet common features: " << kps.size() << endl;
    return lambda;

}
double PoseEstimator::solveScalePnPRANSAC(View *v, const Mat prevPose, map<long, Landmark> landmarkBook, double initial)
{
//    vector<double> lambdaDist;
//    // for each point, compute a lambda
//    for(int i = 0; i < v->getLeftFeatureSet().size(); i++)
//    {
//        KeyPoint kp = v->getLeftFeatureSet().getFeaturePoints()[i];
//        long id = v->getLeftFeatureSet().getIds()[i];
//        if(!landmarkBook.count(id))
//            continue;
//        Point3d p3d = landmarkBook[id].getPoint();
//        double lambda = initial;
//        double *par = &lambda;
//        vector<KeyPoint> kps;
//        vector<Point3d> p3ds;
//        kps.push_back(kp);
//        p3ds.push_back(p3d);
//        const int nEqs = 2;
//        ScaleEstimatorStruct dataStruct(v, prevPose, kps, p3ds);
//        // 2. nonlinear optimization (levenburg marquardt)
//        /* auxiliary parameters */
//        lm_control_struct control = lm_control_double;
//        lm_status_struct  status;
//        
//        control.verbosity = 0;
//        
//        lmmin(1, par, nEqs, &dataStruct, evaluateReprojectionError, &control, &status);
//        lambdaDist.push_back(lambda);
//    }
//    sort(lambdaDist.begin(), lambdaDist.end());
//    rejectDistributionOutliers(lambdaDist);
//    double lambda = median(lambdaDist);
    
    
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
        ScaleEstimatorStruct dataStruct(v, prevPose, kpsRANSAC, p3dsRANSAC);
        
        // 2. nonlinear optimization (levenburg marquardt)
        /* auxiliary parameters */
        lm_control_struct control = lm_control_double;
        lm_status_struct  status;
        control.verbosity = 0;
        
        double lambda = initial;
        double *par = &lambda;
        lmmin(1, par, nEqs, &dataStruct, evaluateReprojectionError, &control, &status);
        
        // test this lambda on entire dataset
        ScaleEstimatorStruct testData(v, prevPose, kps, p3ds);
        evaluateReprojectionError(lambda, &testData, NULL);
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
        ScaleEstimatorStruct testData(v, prevPose, inlierKps, inlierP3ds);
        const int nEqs = int(inlierKps.size()) * 2;
        // 2. nonlinear optimization (levenburg marquardt)
        /* auxiliary parameters */
        lm_control_struct control = lm_control_double;
        lm_status_struct  status;
        control.verbosity = 0;
        
        double *par = &lambda;
        lmmin(1, par, nEqs, &testData, evaluateReprojectionError, &control, &status);
        
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

void computeReprojectionErrorByTrinocular
(const double *par, const int nEqs, const void *data, double *fvec, int *userBreak)
{
    // unpack initial guess
    double lambda = *par;
    // unpack data struct
    TrinocularDataStruct *dataStruct = (TrinocularDataStruct*) data;
    View *v1 = dataStruct->v1;
    View *v2 = dataStruct->v2;
    View *v = dataStruct->v;
    map<long, vector<KeyPoint>> commonFeatures(dataStruct->commonFeatures);
    

    
    Mat pose_v2v = v2->getPose().inv() * v->getPose();
    const Mat pose_v1v2 = v1->getPose().inv() * v2->getPose();
    // multiply by lambda
    pose_v2v.rowRange(0, 3).col(3) = lambda * pose_v2v.rowRange(0, 3).col(3);
    Mat pose_v1v = pose_v1v2 * pose_v2v;
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
    Mat F_v1v = CameraParameters::getIntrinsic().inv().t() * E_v1v * CameraParameters::getIntrinsic().inv();
    Mat F_v2v = CameraParameters::getIntrinsic().inv().t() * E_v2v * CameraParameters::getIntrinsic().inv();
    double totalError = 0.0;
    int i = 0;
    for(map<long, vector<KeyPoint>>::iterator it = commonFeatures.begin(); it != commonFeatures.end(); it++)
    {
        
        KeyPoint kp_v1 = it->second[0], kp_v2 = it->second[1], kp_v = it->second[2];

        Mat p_v1 = Mat::ones(3, 1, CV_64F), p_v2 = Mat::ones(3, 1, CV_64F), p_v = Mat::ones(3, 1, CV_64F);
        p_v1.at<double>(0, 0) = kp_v1.pt.x;
        p_v1.at<double>(1, 0) = kp_v1.pt.y;
        p_v2.at<double>(0, 0) = kp_v2.pt.x;
        p_v2.at<double>(1, 0) = kp_v2.pt.y;
        p_v.at<double>(0, 0) = kp_v.pt.x;
        p_v.at<double>(1, 0) = kp_v.pt.y;
        const double x = p_v.at<double>(0, 0), y = p_v.at<double>(1, 0);
        const Mat epi_v1v = F_v1v * p_v1, epi_v2v = F_v2v * p_v2;
        double a1 = epi_v1v.at<double>(0, 0), b1 = epi_v1v.at<double>(1, 0), c1 = epi_v1v.at<double>(2, 0);
        double a2 = epi_v2v.at<double>(0, 0), b2 = epi_v2v.at<double>(1, 0), c2 = epi_v2v.at<double>(2, 0);
    
        // compute intersection of two epipolar lines (reprojection point)
        double x_hat = (b1 * c2 - b2 * c1) / (a1 * b2 - a2 * b1);
        double y_hat = (a2 * c1 - a1 * c2) / (a1 * b2 - a2 * b1);
        Mat projected_hat(3, 1, CV_64F);
        projected_hat.at<double>(0, 0) = x_hat;
        projected_hat.at<double>(1, 0) = y_hat;
        projected_hat.at<double>(2, 0) = 1;
//        cout << projected_hat << endl;
        Mat projected(3, 1, CV_64F);
        projected.at<double>(0, 0) = x;
        projected.at<double>(1, 0) = y;
        projected.at<double>(2, 0) = 1;
//        cout << projected << endl;
//        cout << "----------" << endl;
        fvec[i++] = projected.at<double>(0, 0) - projected_hat.at<double>(0, 0);
        totalError += (projected.at<double>(0, 0) - projected_hat.at<double>(0, 0))
                        * (projected.at<double>(0, 0) - projected_hat.at<double>(0, 0));
        fvec[i++] = projected.at<double>(1, 0) - projected_hat.at<double>(1, 0);
        totalError += (projected.at<double>(1, 0) - projected_hat.at<double>(1, 0))
        * (projected.at<double>(1, 0) - projected_hat.at<double>(1, 0));
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
    // 1 DoF
    double lambda = 1.0;
    
    double *par = &lambda;

    TrinocularDataStruct dataStruct(v1, v2, v, commonFeatures, 0, 10);
    const int nEqs = int(commonFeatures.size()) * 2;
    // 2. nonlinear optimization (levenburg marquardt)
    /* auxiliary parameters */
    lm_control_struct control = lm_control_double;
    lm_status_struct  status;
    
    control.verbosity = 0;
    
    lmmin(1, par, nEqs, &dataStruct, computeReprojectionErrorByTrinocular, &control, &status);
    
    // re-construct pose
    Mat relativePose = v2->getPose().inv() * v->getPose();
    relativePose.col(3).rowRange(0, 3) = lambda * relativePose.col(3).rowRange(0, 3);
    v->setPose(v2->getPose() * relativePose);
    return v->getPose();
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
                        CameraParameters::focal, CameraParameters::principal, RANSAC, 0.999, 1.0, essentialMatrixStat);
    // reject outliers
    // rejectOutliers(v1, v2, essentialMatrixStat);

    // cout << "essential matrix inliers: " << count << "/" << points1.size() << endl;
    
    // 3. recorver pose (translation up to a scale)
    Mat poseRecoveryStat;
    Mat R, t;
    KeyPoint::convert(newKps1, points1);
    KeyPoint::convert(newKps2, points2);
    recoverPose(E, points2, points1, R, t,
                CameraParameters::focal, CameraParameters::principal, poseRecoveryStat);
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
    v2->setPose(v1->getPose() * poseChange);
    
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
    set<long> inlierList;
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
    double delta = sqrt(REPROJECTION_THRESHOLD * REPROJECTION_THRESHOLD / 2);
    for(map<long, vector<KeyPoint>>::iterator it = commonFeatures.begin(); it != commonFeatures.end(); it++)
    {
        bool isInlier = true;
        totalCount++;
        long id = it->first;
        KeyPoint kp_v1 = it->second[0], kp_v2 = it->second[1], kp_v3 = it->second[2], kp_v4 = it->second[3];
        Point3d p3d = landmarkBook[id].getPoint();
        pair<double, double> err = {0.0, 0.0};
        double reprojectionError = 0.0;
//        err = reproject3DPoint(p3d, pose_v1, kp_v1, false);
//        if(fvec != NULL)
//        {
//            *(fvec++) = err.first < delta ? err.first : delta;
//            *(fvec++) = err.second < delta ? err.second : delta;
//            if(err.first >= delta || err.second >= delta)
//                isInlier = false;
//        }
//        totalError += (err.first * err.first + err.second * err.second);
//        err = reproject3DPoint(p3d, pose_v2, kp_v2, false);
//        if(fvec != NULL)
//        {
//            *(fvec++) = err.first < delta ? err.first : delta;
//            *(fvec++) = err.second < delta ? err.second : delta;
//            if(err.first >= delta || err.second >= delta)
//                isInlier = false;
//        }
        totalError += (err.first * err.first + err.second * err.second);
        err = reproject3DPoint(p3d, pose_v3, kp_v3, false);
        if(fvec != NULL)
        {
            *(fvec++) = huber(err.first, delta, true);
            *(fvec++) = huber(err.second, delta, true);
        }
        reprojectionError = sqrt(err.first * err.first + err.second * err.second);
        if(reprojectionError >= REPROJECTION_THRESHOLD)
            isInlier = false;
        totalError += (err.first * err.first + err.second * err.second);
        err = reproject3DPoint(p3d, pose_v4, kp_v4, false);
        if(fvec != NULL)
        {
            *(fvec++) = huber(err.first, delta, true);
            *(fvec++) = huber(err.second, delta, true);
        }
        if(reprojectionError >= REPROJECTION_THRESHOLD)
            isInlier = false;
        totalError += (err.first * err.first + err.second * err.second);
        if(isInlier)
        {
            inlierCount++;
            inlierList.insert(id);
        }
    }
    inlierRatio = double(inlierCount) / totalCount;
    dataStruct->inlier = inlierRatio;
    dataStruct->error = totalError;
    dataStruct->inlierList = set<long>(inlierList);
}
void computeReprojectionErrorFourFrames
(const double *par, const int nEqs, const void *data, double *fvec, int *userBreak)
{
    // unpack initial guess (of the pose)
    double lambda = *par;
    // compute reprojection error
    computeReprojectionErrorFourFrames(data, lambda, fvec);
}
double PoseEstimator::refineScale(vector<View*> views, map<long, Landmark> &landmarkBook)
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
        if(it->second.size() < 4 || !landmarkBook.count(it->first))
            it = commonFeatures.erase(it);
        else
        {
            ids.push_back(it->first);
            it++;
        }
    }
    cout << "quadruple common features: " << commonFeatures.size() << endl;
    if(commonFeatures.size() < 20)
        return 1.0;
    // lm
    // 1 DoF
    double lambda = .99;
    
    double *par = &lambda;
    
    ScaleRefinementDataStruct *dataStruct = new ScaleRefinementDataStruct(v1, v2, v3, v4, landmarkBook, commonFeatures);
    const int nEqs = int(commonFeatures.size()) * 4 * 2;
    
    // 2. nonlinear optimization (levenburg marquardt)
    /* auxiliary parameters */
    lm_control_struct control = lm_control_double;
    lm_status_struct  status;
    
    control.verbosity = 0;
    
    lmmin(1, par, nEqs, dataStruct, computeReprojectionErrorFourFrames, &control, &status);
    
    return lambda;
}
double PoseEstimator::refineScaleRANSAC(vector<View*> views, map<long, Landmark> &landmarkBook)
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
        if(it->second.size() < 4 || !landmarkBook.count(it->first))
            it = commonFeatures.erase(it);
        else
        {
            ids.push_back(it->first);
            it++;
        }
    }
    if(commonFeatures.size() < 20)
        return 1.0;
    // RANSAC
    srand(time(NULL));
    double maxInlierRatio = 0.0;
    double bestLambda = 1.0;
    set<long> inlierList;
    int N = 100;
    int K = 3;
    for(int i = 0; i < N; i++)
    {
        // generate k random number between 0 and commonFeatures.size()
        map<long, vector<KeyPoint>> commonFeaturesRANSAC;
        for(int k = 0; k < K; k++)
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
        const int nEqs = int(commonFeaturesRANSAC.size()) * 2 * 2;
        // 2. nonlinear optimization (levenburg marquardt)
        /* auxiliary parameters */
        lm_control_struct control = lm_control_double;
        lm_status_struct  status;
        
        control.verbosity = 0;
        
        lmmin(1, par, nEqs, dataStruct, computeReprojectionErrorFourFrames, &control, &status);
        
        
        // test this lambda on entire dataset
        ScaleRefinementDataStruct *completeDataStruct =
                        new ScaleRefinementDataStruct(v1, v2, v3, v4, landmarkBook, commonFeatures);
        computeReprojectionErrorFourFrames(completeDataStruct, lambda, NULL);
        // record only the best set of data points and their corresponding ratio
        double inlierRatio = completeDataStruct->inlier;
        if(maxInlierRatio < inlierRatio)
        {
            maxInlierRatio = inlierRatio;
            bestLambda = lambda;
            inlierList = set<long>(completeDataStruct->inlierList);
        }
    }
    
    
    // final test
    map<long, vector<KeyPoint>> inlierFeatures;
    for(map<long, vector<KeyPoint>>::iterator it = commonFeatures.begin(); it != commonFeatures.end(); it++)
    {
        if(inlierList.count(it->first))
            inlierFeatures[it->first] = vector<KeyPoint>(it->second);
    }
    double lambda = bestLambda;
    
    double *par = &lambda;
    ScaleRefinementDataStruct *dataStruct =
                    new ScaleRefinementDataStruct(v1, v2, v3, v4, landmarkBook, inlierFeatures);
    const int nEqs = int(inlierFeatures.size()) * 2 * 2;
    // 2. nonlinear optimization (levenburg marquardt)
    /* auxiliary parameters */
    lm_control_struct control = lm_control_double;
    lm_status_struct  status;
    
    control.verbosity = 0;
    
    lmmin(1, par, nEqs, dataStruct, computeReprojectionErrorFourFrames, &control, &status);
    
    cout << "quadruple lambda(set of " << commonFeatures.size() << "): " << lambda << endl;
    cout << "final inliers: " << dataStruct->inlier * 100 << "%" << endl;
    
    return lambda;
}
void PoseEstimator::refineScaleMultipleFrames(vector<View*> views, int N)
{
    // optimize N-ples;
    vector<double> scales = {2.07632};
    vector<Mat> relativePoses;
    relativePoses.push_back(views[0]->getPose().inv() * views[1]->getPose());
    Canvas *canvas = new Canvas();
    for(int i = 0; i < views.size() - (N - 1); i++)
    {
        vector<View*> bag;
        for(int j = 0; j < N; j++)
            bag.push_back(views[i + j]);
        const Mat relativePose = bag[1]->getPose().inv() * bag[2]->getPose();
        ViewTracker *localViewTracker = new ViewTracker();
        localViewTracker->addView(bag[0]);
        localViewTracker->addView(bag[1]);
        // triangulation
        localViewTracker->computeLandmarks(true);
        for(int j = 2; j < N; j++)
            localViewTracker->addView(bag[j]);
        
        // solve the scale between 2nd and 3rd frame (adjust later)
        double scale = 1.0;
        if(N == 3)
            scale = solveScalePnPRANSAC(bag[2], bag[1]->getPose(),
                                                    localViewTracker->getLandmarkBook());
        else if(N == 4)
            scale = refineScaleRANSAC(bag, localViewTracker->getLandmarkBook());
        
        scales.push_back(scales.back() * scale);
        relativePoses.push_back(relativePose.clone());
    }
    
    // update poses
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
    for(double lambda : scales)
    {
        cout << lambda << endl;
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
