//
//  Converter.hpp
//  VisualSLAM
//
//  Created by Rong Yuan on 3/10/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#ifndef Converter_h
#define Converter_h

#include <stdio.h>

#include "opencv2/opencv.hpp"
#include "g2o/types/slam3d/vertex_se3.h"

using namespace std;
using namespace cv;
using namespace Eigen;

class Converter
{
public:
    static Mat eigenMatToCvMat(const Matrix<double, 4, 4> &matrix);
    static Point3d vector3dToPoint3d(const Vector3d &p);
};
#endif /* Converter_h */
