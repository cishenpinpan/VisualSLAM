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

Point3d Converter::vector3dToPoint3d(const Vector3d &p)
{
    return {p.x(), p.y(), p.z()};
}
