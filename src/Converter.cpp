//
//  Converter.cpp
//  VisualSLAM
//
//  Created by Rong Yuan on 3/10/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#include "Converter.h"

Mat Converter::eigenMatToCvMat(const Matrix<double, 4, 4> &matrix)
{
    Mat res(4, 4, CV_64F);
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            res.at<double>(i, j) = matrix(i, j);
        }
    }
    return res.clone();
}
Mat Converter::tVecToTx(const Mat tVec)
{
    Mat res = Mat::zeros(3, 3, CV_64F);
    res.at<double>(0, 1) = -tVec.at<double>(2, 0);
    res.at<double>(0, 2) = tVec.at<double>(1, 0);
    res.at<double>(1, 0) = tVec.at<double>(2, 0);
    res.at<double>(1, 2) = -tVec.at<double>(0, 0);
    res.at<double>(2, 0) = -tVec.at<double>(1, 0);
    res.at<double>(2, 1) = tVec.at<double>(0, 0);
    return res.clone();
}
Mat Converter::rotationTranslationToPose(const Mat R, const Mat t)
{
    Mat pose = Mat::eye(4, 4, CV_64F);
    Mat aux = pose(Rect(0, 0, 3, 3));
    R.clone().copyTo(aux);
    aux = pose(Rect(3, 0, 1, 3));
    t.clone().copyTo(aux);
    return pose.clone();
}
Point3d Converter::vector3dToPoint3d(const Vector3d &p)
{
    return {p.x(), p.y(), p.z()};
}
vector<Point2f> Converter::keyPointsToPoint2fs(const vector<KeyPoint> keyPoints)
{
    vector<Point2f> arr;
    for(KeyPoint kp : keyPoints)
    {
        arr.push_back(Point2f(kp.pt.x, kp.pt.y));
    }
    return arr;
}

vector<KeyPoint> Converter::point2fsToKeyPoints(const vector<Point2f> points)
{
    vector<KeyPoint> arr;
    for(Point2f p : points)
    {
        arr.push_back(KeyPoint({p.x, p.y}, 1.f));
    }
    return arr;
}
