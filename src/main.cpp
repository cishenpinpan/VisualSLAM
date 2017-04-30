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

// #include <cvsba/cvsba.h>

#include <fstream>
#include <list>

using namespace cv;
using namespace std;


#define DATASET "KITTI"
#define STEREO true
#define MONOCULAR false

int main( int argc, char** argv )
{
    // create a bunch of strategy class objects
    ViewReader *reader = new ViewReader(DATASET, TRACK, STEREO);
    FeatureExtractor *featureExtractor = new FeatureExtractor();
    FeatureTracker *featureTracker = new FeatureTracker();
    ViewTracker *viewTracker = new ViewTracker();
    PoseEstimator *poseEstimator = new PoseEstimator();
    
    // start monocular odometry
    vector<Mat> Trs;
    vector<View*> views;

    MonocularOdometry *monocularOdometry = new MonocularOdometry();

	cout << "============================Trinocular Stereo=========================" << endl;
    // trinocualr
    monocularOdometry->run(reader, featureExtractor, featureTracker, viewTracker, poseEstimator, views, TRINOCULAR, 0);
    
    // save result to file
    monocularOdometry->save(TRACK, views, "TrinocularStereo");
   	
	cout << "===========================Triangulation Stereo==================" << endl;
    // use the views above
    // triangulation
    monocularOdometry->run(reader, featureExtractor, featureTracker, viewTracker, poseEstimator, views, TRIANGULATION, 0);
    
    // save result to file
    monocularOdometry->save(TRACK, views, "ScalePnPStereo");
	
	cout << "============Triangulation Stereo with 1D RANSAC==================" << endl;
	//triangulation with 1d RANSAC
    monocularOdometry->run(reader, featureExtractor, featureTracker, viewTracker, poseEstimator, views, TRIANGULATION, 1);
	monocularOdometry->save(TRACK, views, "ScalePnPStereo_1DRANSAC");

	cout << "============Trinocular Stereo with 1D RANSAC==================" << endl;
	//triangulation with 1d RANSAC
    monocularOdometry->run(reader, featureExtractor, featureTracker, viewTracker, poseEstimator, views, TRINOCULAR, 1);
	monocularOdometry->save(TRACK, views, "TrinocularStereo_1DRANSAC");

    return 0;
}


