//
//  FeatureTracker.cpp
//  VisualSLAM
//
//  Created by Rong Yuan on 2/24/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#include "ViewTracker.h"

ViewTracker::ViewTracker()
{
    featureTracker = new FeatureTracker();
    featureExtractor = new FeatureExtractor();
    nBundleAdjusted = 0;
}
ViewTracker::~ViewTracker()
{
    ;
}
void ViewTracker::addView(View *v)
{
    views.push_back(v);
    viewBook[v->getId()] = v;
    
}
void ViewTracker::setKeyView(View *v)
{
    keyViews.push_back(v);
    v->setKeyView();
}
void ViewTracker::computeLandmarks(bool initial)
{
    // triangulate feature correspondences in keyframes and update landmark book
    View *prevView = keyViews[keyViews.size() - 2];
    View *currView = keyViews.back();
    Canvas *canvas = new Canvas();
    vector<long> idToRemovePrev, idToRemoveCurr;
    vector<double> depths;
    for(int i = 0; i < currView->getLeftFeatureSet().size(); i++)
    {
        Feature currFeature = currView->getLeftFeatureSet().getFeatureByIndex(i);
        long int id = currFeature.getId();
        if(!prevView->getLeftFeatureSet().hasId(id))
            continue;
        Feature prevFeature = prevView->getLeftFeatureSet().getFeatureById(id);
        // triangulate this pair of points
        Point3d point3d;
        
        triangulatePoint(prevView->getPose(), currView->getPose(),
                         prevFeature.getPoint(), currFeature.getPoint(), point3d);
        Landmark *landmark = new Landmark(point3d, id, {prevView->getId(), currView->getId()});
        // compute descriptor
        Ptr<SURF> extractor = SURF::create();
        Mat descriptor;
        vector<KeyPoint> tmp;
        tmp.push_back(currFeature.getPoint());
        extractor->compute(currView->getImgs()[0], tmp, descriptor);
        landmark->setDescriptor(descriptor);
        // reproject 3d point
        // add to landmark book if this feature is not yet triangulated
        if(!landmarkBook.count(id))
        {
            pair<double, double> err = reproject3DPoint(point3d, prevView->getPose(), prevFeature.getPoint(), false);
            pair<double, double> err2 = reproject3DPoint(point3d, currView->getPose(), currFeature.getPoint(), false);
            double l2NormError = sqrt(err.first * err.first + err.second * err.second);
            double l2NormError2 = sqrt(err2.first * err2.first + err2.second * err2.second);
            if(l2NormError < REPROJECTION_THRESHOLD && l2NormError2 < REPROJECTION_THRESHOLD)
            {
                landmarkBook.insert(make_pair(id, *landmark));
            }
            else
            {
                idToRemovePrev.push_back(id);
                idToRemoveCurr.push_back(id);
            }
        }
        else
        {
            point3d = Point3d(landmarkBook[id].point3d);
            pair<double, double> err = reproject3DPoint(point3d, currView->getPose(), currFeature.getPoint(), false);
            double l2NormError = sqrt(err.first * err.first + err.second * err.second);
//            if(l2NormError > REPROJECTION_THRESHOLD)
//                idToRemoveCurr.push_back(id);
        }
    }
    // remove
    for(int i = 0; i < idToRemovePrev.size(); i++)
    {
        prevView->removeLeftFeatureById(idToRemovePrev[i]);
    }
    for(int i = 0; i < idToRemoveCurr.size(); i++)
    {
        currView->removeLeftFeatureById(idToRemoveCurr[i]);
    }
    
}

vector<View*> ViewTracker::getViews()
{
    return views;
}
View* ViewTracker::getLastView()
{
    return views.back();
}

View* ViewTracker::popLastView()
{
    View* temp = views.back();
    views.erase(views.end() - 1);
    return temp;
}

void ViewTracker::bundleAdjust(int option, bool global)
{
    vector<View*> viewsForBA;
    if(global)
        viewsForBA = vector<View*>(keyViews);
    else
    {
        viewsForBA = vector<View*>(getLastTwoKeyViews());
        viewsForBA.push_back(views.back());
    }
    if(viewsForBA.size() < 2)
        return ;
    
    // setting up g2o solver
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver =
            new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
    
    // setting up camer parameters
    double focalLength = CameraParameters::focal;
    Vector2d principalPoint(CameraParameters::principal.x, CameraParameters::principal.y);
    vector<g2o::SE3Quat, aligned_allocator<g2o::SE3Quat> > true_poses;
    g2o::CameraParameters * camParams = new g2o::CameraParameters (focalLength, principalPoint, 0.);
    camParams->setId(0);
    if (!optimizer.addParameter(camParams)) {
        assert(false);
    }
    
    // setting up camera poses as vertices
    int start = max(0, int(viewsForBA.size()) - BUNDLE_ADJUSTMENT_LENGTH);
    vector<g2o::SE3Quat, aligned_allocator<g2o::SE3Quat> > truePoses;
    for (int i = start; i < viewsForBA.size(); i++)
    {
        Mat pose = viewsForBA[i]->getPose();
        pose = pose.inv();
        Vector3d t(pose.at<double>(0, 3), pose.at<double>(1, 3), pose.at<double>(2, 3));
        vector<double> quat = rot2quat(pose(Rect(0, 0, 3, 3)));
        Quaterniond q = Quaterniond(quat[0], quat[1], quat[2], quat[3]);
        g2o::SE3Quat g2oPose(q,t);
        g2o::VertexSE3Expmap * v_se3 = new g2o::VertexSE3Expmap();
        v_se3->setId(int(viewsForBA[i]->getId()));
        if((i == start || option == STRUCTURE_ONLY) && i != int(viewsForBA.size()) - 1)
        {
            v_se3->setFixed(true);
        }
        v_se3->setEstimate(g2oPose);
        optimizer.addVertex(v_se3);
        truePoses.push_back(g2oPose);
    }
    
    // setting up landmarks as vertices
    Canvas *canvas = new Canvas();
    const float thHuber = sqrt(5.991);
    vector<int> counts(BUNDLE_ADJUSTMENT_LENGTH, 0);
    vector<double> errors(BUNDLE_ADJUSTMENT_LENGTH, 0.0);
    for (map<long, Landmark>::iterator landmarkIter = landmarkBook.begin();
                        landmarkIter != landmarkBook.end(); landmarkIter++)
    {
        long id = landmarkIter->first;
        Landmark landmark = landmarkIter->second;
        Vector3d point3d = Vector3d(landmark.point3d.x, landmark.point3d.y, landmark.point3d.z);
        g2o::VertexSBAPointXYZ * v_p = new g2o::VertexSBAPointXYZ();
        v_p->setId(int(id));
        v_p->setMarginalized(true);
        v_p->setEstimate(point3d);
        if(option == MOTION_ONLY)
            v_p->setFixed(true);
        optimizer.addVertex(v_p);
        for (int j = start; j < viewsForBA.size(); j++)
        {
            
            // Add edges. See the following passage.
            if(!viewsForBA[j]->getLeftFeatureSet().hasId(id))
                continue;
            Feature feature = viewsForBA[j]->getLeftFeatureSet().getFeatureById(id);
            pair<double, double> err = reproject3DPoint(Point3d(point3d.x(), point3d.y(), point3d.z()),
                                                        viewsForBA[j]->getPose(), feature.getPoint(), false);
            double l2NormError = sqrt(err.first * err.first + err.second * err.second);
            View *vFrom1 = viewBook[landmark.from.first], *vFrom2 = viewBook[landmark.from.second];
            // plot matches
            Vector2d z = Vector2d(feature.getPoint().pt.x, 376 - feature.getPoint().pt.y);
            g2o::EdgeProjectXYZ2UV * e = new g2o::EdgeProjectXYZ2UV();
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>
                         (optimizer.vertices().find(int(viewsForBA[j]->getId()))->second));
            e->setMeasurement(z);
            e->information() = Matrix2d::Identity();
            e->setParameterId(0, 0);
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuber);
            optimizer.addEdge(e);
            counts[j - start]++;
            errors[j - start] += l2NormError;
            
            // draw matches
            if(l2NormError > 2)
            {
                vector<Point2f> p1, p2;
                KeyPoint::convert({vFrom1->getLeftFeatureSet().getFeatureById(id).getPoint()}, p1);
                KeyPoint::convert({vFrom2->getLeftFeatureSet().getFeatureById(id).getPoint()}, p2);
            }
            
        }
    }
//    for (int j = start; j < keyViews.size(); j++)
//    {
//        cout << "Frame (" << keyViews[j]->getId() << "): " << counts[j - start] << "--> " <<
//                errors[j - start] / counts[j - start] << endl;
//    }
    optimizer.setVerbose(false);
    // optimizer.save("/Users/orangechicken/Desktop/SLAM/g2o_data/firstInput.g2o");
    
    // start optimization
    optimizer.initializeOptimization();
    string type;
    if(option == MOTION_ONLY)
        type = "MOTION_ONLY";
    if(option == STRUCTURE_ONLY)
        type = "STRUCTURE_ONLY";
    if(option == MOTION_STRUCTURE)
        type = "MOTION_STRUCTURE";
    cout << "Performing " << type << " BA:" << endl;
    optimizer.optimize(10);
    
    // save result
    // optimizer.save("/Users/orangechicken/Desktop/SLAM/g2o_data/firstOutput.g2o");
    cout << type << " BA Finished" << endl;
    
    // update 3d landmarks and poses of views
    // views
    for(int i = start; i < viewsForBA.size(); i++)
    {
        long id = viewsForBA[i]->getId();
        g2o::HyperGraph::VertexIDMap::iterator it = optimizer.vertices().find(int(id));
        if(it == optimizer.vertices().end())
            continue;
        g2o::VertexSE3Expmap *v_se3 = dynamic_cast<g2o::VertexSE3Expmap*>(it->second);
        g2o::SE3Quat se3quat = v_se3->estimate();
        Matrix<double, 4, 4> pose = se3quat.to_homogeneous_matrix();
        // reject large motion
        Mat currPose = viewsForBA[i]->getPose(), estimatedPose = Converter::eigenMatToCvMat(pose).inv();
        double xDiff = abs(currPose.at<double>(0, 3) - estimatedPose.at<double>(0, 3));
        double yDiff = abs(currPose.at<double>(1, 3) - estimatedPose.at<double>(1, 3));
        double zDiff = abs(currPose.at<double>(2, 3) - estimatedPose.at<double>(2, 3));
        
//        if(xDiff > 1 || yDiff > 1 || zDiff > 1)
//        {
//            cout << "Large change after BA." << endl;
//            return ;
//        }
        viewsForBA[i]->setPose(estimatedPose);
    }
    // landmarks
    for(map<long, Landmark>::iterator landmarkIter = landmarkBook.begin();
                landmarkIter != landmarkBook.end(); landmarkIter++)
    {
        long id = landmarkIter->first;
        g2o::HyperGraph::VertexIDMap::iterator it = optimizer.vertices().find(int(id));
        if(it == optimizer.vertices().end())
            continue;
        g2o::VertexSBAPointXYZ *v_p = dynamic_cast<g2o::VertexSBAPointXYZ*>(it->second);
        Point3d currPoint3d = landmarkIter->second.getPoint(),
                estimatedPoint3d = Converter::vector3dToPoint3d(v_p->estimate());
        double xDiff = abs(currPoint3d.x - estimatedPoint3d.x);
        double yDiff = abs(currPoint3d.y - estimatedPoint3d.y);
        double zDiff = abs(currPoint3d.z - estimatedPoint3d.z);
        landmarkIter->second.setPoint(estimatedPoint3d);
    }
    
    // update nBundleAdjust
    nBundleAdjusted = int(viewsForBA.size());
}
