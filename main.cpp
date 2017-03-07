#include "ViewReader.h"
#include "CameraParameters.h"
#include "View.h"
#include "ViewTracker.h"
#include "PoseEstimator.h"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/core/affine.hpp"
#include "opencv2/calib3d/calib3d_c.h"
#include "opencv2/plot.hpp"
#include "camera.hpp"
#include <cvsba/cvsba.h>


#include <fstream>

using namespace cv;
using namespace std;

vector<View*> track;
vector<Mat> Trs;
string dataset = "00";

vector<vector<double>> readGroundTruth(int numPoses)
{
    vector<vector<double>> gt;
    ifstream input;
    input.open("/Users/orangechicken/Documents/MATLAB/KITTI/GroundTruth/poses/" + dataset + ".txt");
    for(int i = 0; i <= numPoses; i++)
    {
        vector<double> row(12, 0.0);
        for(int j = 0; j < 12; j++)
        {
            input >> row[j];
        }
        gt.push_back(row);
    }
    input.close();
    return gt;
}


void printPoses(ViewTracker* vt)
{
    vector<View*> views = vt->getViews();
    for(int i = 0; i < views.size(); i++)
    {
        cout << views[i]->getPose() << endl;
    }
}

int main( int argc, char** argv )
{
    int numPoses = 100;
    // read ground truth
    vector<vector<double>> gt = readGroundTruth(numPoses);

    Mat Tr = Mat::eye(4, 4, CV_64F);
    
    ViewReader reader = ViewReader("KITTI", dataset, true);
    // initial image
    vector<Mat> images = reader.next();

    View *prevView = new View(images);
    prevView->setPose(Mat::eye(4, 4, CV_64F));
    prevView->setGroundTruth(Mat::eye(4, 4, CV_64F));
    
    list<detail::CameraParams> cameraPoses;
    
    // build an initial feature tracker
    Trs.push_back(Tr);
    
    Mat prevPose = Mat::eye(4, 4, CV_64F);
    Mat currPose = Mat::eye(4, 4, CV_64F);
    
    PoseEstimator *poseEstimator = new PoseEstimator();
    for(int i = 0; i < numPoses; i++)
    {

        if((i + 1) % 20 == 0)
            cout << "frame: " << i + 1 << endl;
        // 1. read in next image
        images = reader.next();
        vector<Mat> currImgs = images;
        
        // 2. initialize next view
        View *currView = new View(currImgs);
        
        
        // 3. estimate pose from monocular odometry
        // 3.1 estimate R and t
        Mat poseChange = poseEstimator->estimatePose(prevView, currView);
        
        // 4. use ground truth scale if necessary
        double scale = pow(gt[i][3] - gt[i + 1][3], 2) +
                    pow(gt[i][7] - gt[i + 1][7], 2) +
                    pow(gt[i][11] - gt[i + 1][11], 2);
        scale = sqrt(scale);
//        t = scale * t;
        
        prevPose.at<double>(0, 0) = gt[i][0];
        prevPose.at<double>(0, 1) = gt[i][1];
        prevPose.at<double>(0, 2) = gt[i][2];
        prevPose.at<double>(1, 0) = gt[i][4];
        prevPose.at<double>(1, 1) = gt[i][5];
        prevPose.at<double>(1, 2) = gt[i][6];
        prevPose.at<double>(2, 0) = gt[i][8];
        prevPose.at<double>(2, 1) = gt[i][9];
        prevPose.at<double>(2, 2) = gt[i][10];
        prevPose.at<double>(0, 3) = gt[i][3];
        prevPose.at<double>(1, 3) = gt[i][7];
        prevPose.at<double>(2, 3) = gt[i][11];
        
        currPose.at<double>(0, 0) = gt[i + 1][0];
        currPose.at<double>(0, 1) = gt[i + 1][1];
        currPose.at<double>(0, 2) = gt[i + 1][2];
        currPose.at<double>(1, 0) = gt[i + 1][4];
        currPose.at<double>(1, 1) = gt[i + 1][5];
        currPose.at<double>(1, 2) = gt[i + 1][6];
        currPose.at<double>(2, 0) = gt[i + 1][8];
        currPose.at<double>(2, 1) = gt[i + 1][9];
        currPose.at<double>(2, 2) = gt[i + 1][10];
        currPose.at<double>(0, 3) = gt[i + 1][3];
        currPose.at<double>(1, 3) = gt[i + 1][7];
        currPose.at<double>(2, 3) = gt[i + 1][11];

        Mat groundTruthPose = prevPose.inv() * currPose;
        
       // 5 skipping frame conditions
        currView->setGroundTruth(currPose.clone());
        double lambda = poseChange.at<double>(2, 3);
//        cout << poseChange(Rect(3, 0 ,1 ,3)) << endl;
//        cout << groundTruthPose(Rect(3, 0, 1, 3)) << endl;
        if(scale <= 0.1)
        {
            cout << "Bad estimation(slow motion)." << endl;
            Tr = Tr * groundTruthPose;
        }
        else if(lambda >= 5)
        {
            cout << "Bad estimation(lambda too big)." << endl;
            Tr = Tr * groundTruthPose;
        }
        else if(abs(poseChange.at<double>(0, 3)) >= 0.4 * abs(poseChange.at<double>(2, 3)) ||
                abs(poseChange.at<double>(1, 3)) >= 0.4 * abs(poseChange.at<double>(2, 3)))
        {
            cout << "Bad estimation(horizontal move)." << endl;
            Tr = Tr * groundTruthPose;
        }
        else
        {
            Tr = Tr * poseChange;
        }
        
        Trs.push_back(Tr.clone());
        currView->setPose(Tr.clone());
        
        // 8. update view
        prevView = currView;
    }
    // handle remaining views in view tracker
    // viewTracker->bundleAdjust();
    // vector<View*> bundleAdjustedViews = viewTracker->getViews();
    // Trs.push_back(Tr);
    // delete viewTracker;
    
    // write poses to file
    ofstream output;
    output.open("/Users/orangechicken/Desktop/SLAM/KITTI_" + dataset + "_MonoLeft.output");
    for(int i = 0; i < Trs.size(); i++)
    {
        Mat pose = Trs[i];
        for(int row = 0; row < 4; row++)
        {
            for(int col = 0; col < 4; col++)
            {
                if(col != 0 || row != 0) // first
                    output << " ";
                output << double(pose.at<double>(row, col));
                if(col == 3 && row == 3 && i + 1 != Trs.size())
                    output << "\n";
                
            }
        }
    }
    output.close();
    return 0;
}


