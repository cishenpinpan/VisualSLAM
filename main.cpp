#include "ViewReader.h"
#include "CameraParameters.h"
#include "View.h"
#include "ViewTracker.h"
#include "PoseEstimator.h"
#include "MonocularOdometry.h"
#include "SystemParameters.h"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/core/affine.hpp"
#include "opencv2/calib3d/calib3d_c.h"
#include "opencv2/plot.hpp"

#include <cvsba/cvsba.h>

#include <fstream>
#include <list>

using namespace cv;
using namespace std;


#define DATASET "KITTI"
#define TRACK "00"
#define MONOCULAR false

int main( int argc, char** argv )
{
    // create a bunch of strategy class objects
    ViewReader *reader = new ViewReader(DATASET, TRACK, MONOCULAR);
    FeatureExtractor *featureExtractor = new FeatureExtractor();
    FeatureTracker *featureTracker = new FeatureTracker();
    ViewTracker *viewTracker = new ViewTracker();
    PoseEstimator *poseEstimator = new PoseEstimator();
    
    // start monocular odometry
    vector<Mat> Trs;
    vector<View*> views;
    MonocularOdometry *monocularOdometry = new MonocularOdometry();
    monocularOdometry->run(reader, featureExtractor, featureTracker, viewTracker, poseEstimator, Trs, views);
    
    // save result to file
    monocularOdometry->save(TRACK, views);
    
    return 0;
}


