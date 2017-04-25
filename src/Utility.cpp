//
//  Utility.cpp
//  VisualSLAM
//
//  Created by Rong Yuan on 3/4/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#include "Utility.h"

IdGenerator* IdGenerator::instance = NULL;

void triangulatePoint(const Mat pose1, const Mat pose2, const KeyPoint point1, const KeyPoint point2, Point3d& point3d)
{
    const Mat globalR = pose1(Rect(0, 0, 3, 3));
    const Mat globalT = pose1(Rect(3, 0, 1, 3));
    const Mat relativePose = pose1.inv() * pose2;
    const Mat relativeR = relativePose(Rect(0, 0, 3, 3));
    const Mat relativeT = relativePose(Rect(3, 0, 1, 3));
    const Mat invK = CameraParameters::getIntrinsic().inv();
    
    Mat homo1(3, 1, CV_64F), homo2(3, 1, CV_64F);
    homo1.at<double>(0, 0) = point1.pt.x;
    homo1.at<double>(1, 0) = point1.pt.y;
    homo1.at<double>(2, 0) = 1.0;
    homo2.at<double>(0, 0) = point2.pt.x;
    homo2.at<double>(1, 0) = point2.pt.y;
    homo2.at<double>(2, 0) = 1.0;
    homo1 = invK * homo1;
    homo2 = invK * homo2;
    
    Mat homo1x = Mat::zeros(3, 3, CV_64F), homo2x = Mat::zeros(3, 3, CV_64F);
    homo1x.at<double>(0, 1) = -homo1.at<double>(2, 0);
    homo1x.at<double>(0, 2) = homo1.at<double>(1, 0);
    homo1x.at<double>(1, 0) = homo1.at<double>(2, 0);
    homo1x.at<double>(1, 2) = -homo1.at<double>(0, 0);
    homo1x.at<double>(2, 0) = -homo1.at<double>(1, 0);
    homo1x.at<double>(2, 1) = homo1.at<double>(0, 0);
    homo2x.at<double>(0, 1) = -homo2.at<double>(2, 0);
    homo2x.at<double>(0, 2) = homo2.at<double>(1, 0);
    homo2x.at<double>(1, 0) = homo2.at<double>(2, 0);
    homo2x.at<double>(1, 2) = -homo2.at<double>(0, 0);
    homo2x.at<double>(2, 0) = -homo2.at<double>(1, 0);
    homo2x.at<double>(2, 1) = homo2.at<double>(0, 0);
    
    const Mat ext1 = pose1.inv(), ext2 = pose2.inv();
    Mat v1 = homo1x * ext1(Rect(0, 0, 4, 3)), v2 = homo2x * ext2(Rect(0, 0, 4, 3));
    
    Mat A(6, 4, CV_64F);
    Mat aux = A(Rect(0, 0, 4, 3));
    v1.copyTo(aux);
    aux = A(Rect(0, 3, 4, 3));
    v2.copyTo(aux);
    
    SVD svdMat(A);
    Mat V;
    transpose(svdMat.vt, V);
    
    Mat point3dVec(3, 1, CV_64F);
    point3dVec.at<double>(0, 0) = V.at<double>(0, 3) / V.at<double>(3, 3);
    point3dVec.at<double>(1, 0) = V.at<double>(1, 3) / V.at<double>(3, 3);
    point3dVec.at<double>(2, 0) = V.at<double>(2, 3) / V.at<double>(3, 3);
    
//    cout << point3dVec << endl;
//    
//    pair<double, double> err1, err2;
//    err1 = reproject3DPoint(Point3d(point3dVec), pose1, point1, false);
//    err2 = reproject3DPoint(Point3d(point3dVec), pose2, point2, false);
//    cout << "err1: (" << err1.first << ", " << err1.second << ")";
//    cout << " -> " << sqrt(err1.first * err1.first + err1.second * err1.second) << endl;
//    cout << "err2: (" << err2.first << ", " << err2.second << ")";
//    cout << " -> " << sqrt(err2.first * err2.first + err2.second * err2.second) << endl;
//    cout << endl;
//    
//    Mat c1 = homo1;
//    Mat c2 = -relativeR * homo2;
//    Mat M(3, 2, CV_64F);
//    Mat tmp = M(Rect(0, 0, 1, 3));
//    c1.copyTo(tmp);
//    tmp = M(Rect(1, 0, 1, 3));
//    c2.copyTo(tmp);
//    Mat sol(2, 1, CV_64F);
//    sol = M.inv(DECOMP_SVD) * relativeT;
//    cout << relativeT << endl;
//    
//    Mat w1 = sol.at<double>(0, 0) * homo1;
//    Mat w2 = relativeR * sol.at<double>(1, 0) * homo2 + relativeT;
//    
//    point3dVec = (w1 + w2) / 2;
//    point3dVec = globalR * point3dVec + globalT;
//    
//    err1 = reproject3DPoint(Point3d(point3dVec), pose1, point1, false);
//    err2 = reproject3DPoint(Point3d(point3dVec), pose2, point2, false);
//    cout << "err1: (" << err1.first << ", " << err1.second << ")";
//    cout << " -> " << sqrt(err1.first * err1.first + err1.second * err1.second) << endl;
//    cout << "err2: (" << err2.first << ", " << err2.second << ")";
//    cout << " -> " << sqrt(err2.first * err2.first + err2.second * err2.second) << endl;
    
    point3d = Point3d(point3dVec);
}
void triangulatePoints(Mat globalPose, Mat relativePose, vector<Point2d> points1,
                                    vector<Point2d> points2, vector<Point3d> &points3D)
{
    vector<Point2f> _points1, _points2;
    for(int i = 0; i < points1.size(); i++)
    {
        _points1.push_back(Point2f(points1[i]));
        _points2.push_back(Point2f(points2[i]));
    }
    triangulatePoints(globalPose, relativePose, _points1, _points2, points3D);
}

void triangulatePoints(Mat globalPose, Mat relativePose, vector<Point2f> points1,
                                    vector<Point2f> points2, vector<Point3d> &points3D)
{
    if(points1.size() != points2.size())
    {
        cout << "Number of feature not equal." << endl;
        return ;
    }
    Mat K = CameraParameters::getIntrinsic();
    Mat invK = K.inv();
    Mat globalR = globalPose(Rect(0, 0, 3, 3));
    Mat globalT = globalPose(Rect(3, 0, 1, 3));
    Mat relativeR = relativePose(Rect(0, 0, 3, 3));
    Mat relativeT = relativePose(Rect(3, 0, 1, 3));
    for(int i = 0; i < points1.size(); i++)
    {
        Mat homo1(3, 1, CV_64F), homo2(3, 1, CV_64F);
        homo1.at<double>(0, 0) = points1[i].x;
        homo1.at<double>(1, 0) = points1[i].y;
        homo1.at<double>(2, 0) = 1.0;
        homo2.at<double>(0, 0) = points2[i].x;
        homo2.at<double>(1, 0) = points2[i].y;
        homo2.at<double>(2, 0) = 1.0;
        homo1 = invK * homo1;
        homo2 = invK * homo2;
        cout << homo1 << endl;
        cout << homo2 << endl;
        Mat v1 = homo1;
        Mat v2 = -relativeR * homo2;
        Mat A(3, 2, CV_64F);
        Mat aux = A(Rect(0, 0, 1, 3));
        v1.copyTo(aux);
        aux = A(Rect(1, 0, 1, 3));
        v2.copyTo(aux);
        Mat sol(2, 1, CV_64F);
        sol = A.inv(DECOMP_SVD) * relativeT;
        Point3d point3d(0.0, 0.0, 0.0);
        Mat w1 = sol.at<double>(0, 0) * homo1;
        Mat w2 = relativeR * sol.at<double>(1, 0) * homo2 + relativeT;
        Mat point3dVec = (w1 + w2) / 2;
        point3dVec = globalR * point3dVec + globalT;
        point3d = Point3d(point3dVec);
        points3D.push_back(point3d);
    }
}

vector<double> rot2quat(const Mat R)
{
    double r11 = R.at<double>(0 ,0);
    double r12 = R.at<double>(0 ,1);
    double r13 = R.at<double>(0 ,2);
    double r21 = R.at<double>(1 ,0);
    double r22 = R.at<double>(1 ,1);
    double r23 = R.at<double>(1 ,2);
    double r31 = R.at<double>(2 ,0);
    double r32 = R.at<double>(2 ,1);
    double r33 = R.at<double>(2 ,2);
    double qw = sqrt(1 + r11 + r22 + r33) / 2.0;
    double qx = (r32 - r23) / (4 * qw);
    double qy = (r13 - r31) / (4 * qw);
    double qz = (r21 - r12) / (4 * qw);
    return {qw, qx, qy, qz};
}

pair<double, double> reproject3DPoint(Point3d point3d, Mat pose, KeyPoint point2d, bool L2Norm)
{
    Mat R = pose(Rect(0, 0, 3, 3)).clone();
    Mat T = pose(Rect(3, 0, 1, 3)).clone();
    Mat gammaW = Mat(point3d);
    Mat gammaC = R.inv() * (gammaW - T);
    if(gammaC.at<double>(2, 0) == 0)
    {
        cout << gammaW << endl;
        cout << R << T << endl;
        cout << endl;
    }
    Mat homoC = gammaC / gammaC.at<double>(2, 0);
    Mat pixelC = CameraParameters::getIntrinsic() * homoC;
    pair<double, double> error = {0.0, 0.0};
    
    if(L2Norm)
    {
        error.first = (point2d.pt.x - pixelC.at<double>(0, 0)) * (point2d.pt.x - pixelC.at<double>(0, 0));
        error.second = (point2d.pt.y - pixelC.at<double>(1, 0)) * (point2d.pt.y - pixelC.at<double>(1, 0));

    }
    else
    {
        error.first = point2d.pt.x - pixelC.at<double>(0, 0);
        error.second = point2d.pt.y - pixelC.at<double>(1, 0);
    }
    // cout << pixelC << endl;
    return error;
}
double projectEpipolarLine(const Mat E, Point2f point1, Point2f point2)
{
    Mat pixel1 = Mat::zeros(3, 1, CV_64F), pixel2 = Mat::zeros(3, 1, CV_64F);
    pixel1.at<double>(0, 0) = point1.x;
    pixel1.at<double>(1, 0) = point1.y;
    pixel1.at<double>(2, 0) = 1;
    pixel2.at<double>(0, 0) = point2.x;
    pixel2.at<double>(1, 0) = point2.y;
    pixel2.at<double>(2, 0) = 1;
    Mat F = CameraParameters::getIntrinsic().inv().t() * E * CameraParameters::getIntrinsic().inv();
    Mat line = F * pixel2;
    double A = line.at<double>(0, 0);
    double B = line.at<double>(1, 0);
    double C = line.at<double>(2, 0);
    double dist = (A * point1.x + B * point1.y + C) / sqrt(A * A + B * B);
    return dist;
}
double reproject3DPoints(vector<Point3d> points3D, Mat pose, vector<KeyPoint> points2D)
{
    double totalError = 0.0;
    for(int i = 0; i < points3D.size(); i++)
    {
        pair<double, double> error = reproject3DPoint(points3D[i], pose, points2D[i], true);
        totalError += (error.first + error.second);
    }
    return totalError;
}

Mat getEssentialMatrix(const Mat _R, const Mat _t)
{
    Mat R = _R.clone(), t = _t.clone();
    R = R.inv();
    t = -R * t;
    Mat tx = Mat::zeros(3, 3, CV_64F);
    tx.at<double>(0, 1) = -t.at<double>(2, 0);
    tx.at<double>(0, 2) = t.at<double>(1, 0);
    tx.at<double>(1, 0) = t.at<double>(2, 0);
    tx.at<double>(1, 2) = -t.at<double>(0, 0);
    tx.at<double>(2, 0) = -t.at<double>(1, 0);
    tx.at<double>(2, 1) = t.at<double>(0, 0);
    Mat E = tx * R;
    return E.clone();
}

Mat anglesToRotationMatrix(const Mat &theta)
{
    // Calculate rotation about x axis
    double thetaX = theta.at<double>(0, 0);
    double thetaY = theta.at<double>(1, 0);
    double thetaZ = theta.at<double>(2, 0);
    Mat R_x = (Mat_<double>(3,3) <<
               1,       0,              0,
               0,       cos(thetaX),   -sin(thetaX),
               0,       sin(thetaX),   cos(thetaX)
               );
    
    // Calculate rotation about y axis
    Mat R_y = (Mat_<double>(3,3) <<
               cos(thetaY),    0,      sin(thetaY),
               0,               1,      0,
               -sin(thetaY),   0,      cos(thetaY)
               );
    
    // Calculate rotation about z axis
    Mat R_z = (Mat_<double>(3,3) <<
               cos(thetaZ),    -sin(thetaZ),      0,
               sin(thetaZ),    cos(thetaZ),       0,
               0,               0,                  1);
    
    
    // Combined rotation matrix
    Mat R = R_z * R_y * R_x;
    
    return R.clone();
}
double median(const vector<double> &arr)
{
    vector<double> tmp(arr);
    sort(tmp.begin(), tmp.end());
    return tmp[tmp.size() / 2];
}
double mean(const vector<double> &arr)
{
    double sum = 0.0;
    for(double d : arr)
    {
        sum += d;
    }
    double mean = sum / arr.size();
    return mean;
}
double standardDeviation(const vector<double> &arr)
{
    double avg = mean(arr);
    double sigma = 0.0;
    for(double d : arr)
    {
        sigma += (d - avg) * (d - avg);
    }
    sigma /= arr.size();
    sigma = sqrt(sigma);
    return sigma;
}
void rejectDistributionOutliers(vector<double> &arr)
{
    vector<double> tmp;
    double avg = median(arr);
    double sigma = standardDeviation(arr);
    for(double d : arr)
    {
        if(abs(d - avg) < sigma)
            tmp.push_back(d);
    }
    arr = vector<double>(tmp);
}
double huber(double err, double delta, bool L1Norm)
{
    double huberLoss = 0.0;
    if(abs(err) <= delta)
    {
        huberLoss = 0.5 * err * err;
    }
    else
    {
        huberLoss = delta * (abs(err) - 0.5 * delta);
    }
    return min(abs(err), delta);
    return L1Norm ? sqrt(huberLoss) : huberLoss;
}

