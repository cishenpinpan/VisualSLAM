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
void ViewTracker::computeLandmarks()
{
    // triangulate feature correspondences and update landmark book
    // starting from second frame coming in
    if(views.size() < 2)
        return ;
    View *prevView = views[views.size() - 2];
    View *currView = views.back();
    for(int i = 0; i < currView->getLeftFeatureSet().size(); i++)
    {
        Feature currFeature = currView->getLeftFeatureSet().getFeatureByIndex(i);
        long int id = currFeature.getId();
        if(!prevView->getIdBook().count(id))
            continue;
        int prevFeatureIndex = prevView->getIdBook()[id];
        Feature prevFeature = prevView->getLeftFeatureSet().getFeatureByIndex(prevFeatureIndex);
        // triangulate this pair of points
        Point3d point3d;
        triangulatePoint(prevView->getPose(), currView->getPose(),
                         prevFeature.getPoint(), currFeature.getPoint(), point3d);
        // reproject 3d point
        pair<double, double> err = reproject3DPoint(point3d, prevView->getPose(), prevFeature.getPoint(), false);
        // add to landmark book if this feature is not yet triangulated
        if(!landmarkBook.count(id))
        {
            landmarkBook.insert(make_pair(id, Landmark(point3d, id)));
        }
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


void ViewTracker::bundleAdjust()
{
    Canvas *canvas = new Canvas();
    int c1 = 0, c2 = 5;
    View *v1 = views[c1], *v2 = views[c2];
    FeatureSet fs1 = v1->getLeftFeatureSet(), fs2 = v2->getLeftFeatureSet();
    vector<Point2f> points1, points2;
    for(int i = 0; i < fs2.size(); i++)
    {
        Feature f2 = fs2.getFeatureByIndex(i);
        Feature f1 = fs1.getFeatureByIndex(v1->getIdBook()[f2.getId()]);
        points1.push_back(f1.getPoint());
        points2.push_back(f2.getPoint());
    }
    
//    canvas->drawFeatureMatches(v1->getImgs()[0], v2->getImgs()[0],
//                               points1, points2);
    
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
    vector<g2o::SE3Quat, aligned_allocator<g2o::SE3Quat> > truePoses;
    for (int i = 0; i < views.size(); i++)
    {
        Mat pose = views[i]->getPose();
        pose = pose.inv();
        Vector3d t(pose.at<double>(0, 3), pose.at<double>(1, 3), pose.at<double>(2, 3));
        vector<double> quat = rot2quat(pose(Rect(0, 0, 3, 3)));
        Quaterniond q = Quaterniond(quat[0], quat[1], quat[2], quat[3]);
        g2o::SE3Quat g2oPose(q,t);
        g2o::VertexSE3Expmap * v_se3 = new g2o::VertexSE3Expmap();
        v_se3->setId(int(views[i]->getId()));
        if(i < 1)
        {
            v_se3->setFixed(true);
        }
        v_se3->setEstimate(g2oPose);
        optimizer.addVertex(v_se3);
        truePoses.push_back(g2oPose);
    }
    
    // setting up landmarks as vertices
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
        optimizer.addVertex(v_p);
        for (int j = 0; j < views.size(); j++)
        {
            // Add edges. See the following passage.
            if(!views[j]->getIdBook().count(id))
                continue;
            int featureIndex =  views[j]->getIdBook()[id];
            Feature feature = views[j]->getLeftFeatureSet().getFeatureByIndex(featureIndex);
            pair<double, double> err = reproject3DPoint(Point3d(point3d.x(), point3d.y(), point3d.z()),
                                                        views[j]->getPose(), feature.getPoint(), false);
            cout << "view(" << j << ")" << endl;
            cout << "(" << err.first << ", " << err.second << ")" << endl;
            Vector2d z = Vector2d(feature.getPoint().x, 376 - feature.getPoint().y);
            g2o::EdgeProjectXYZ2UV * e = new g2o::EdgeProjectXYZ2UV();
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>
                         (optimizer.vertices().find(int(views[j]->getId()))->second));
            e->setMeasurement(z);
            e->information() = Matrix2d::Identity();
            e->setParameterId(0, 0);
            optimizer.addEdge(e);
        }
    }
    optimizer.setVerbose(false);
    optimizer.save("/Users/orangechicken/Desktop/SLAM/g2o_data/firstInput.g2o");
    
    // start optimization
    optimizer.initializeOptimization();
    cout << "Performing full BA:" << endl;
    optimizer.optimize(10);
    
    // save result
    optimizer.save("/Users/orangechicken/Desktop/SLAM/g2o_data/firstOutput.g2o");
    cout << "BA Finished" << endl;
    
    // update 3d landmarks and poses of views
    // views
    for(int i = 0; i < views.size(); i++)
    {
        long id = views[i]->getId();
        g2o::HyperGraph::VertexIDMap::iterator it = optimizer.vertices().find(int(id));
        if(it == optimizer.vertices().end())
            continue;
        g2o::VertexSE3Expmap *v_se3 = dynamic_cast<g2o::VertexSE3Expmap*>(it->second);
        g2o::SE3Quat se3quat = v_se3->estimate();
        Matrix<double, 4, 4> pose = se3quat.to_homogeneous_matrix();
        views[i]->setPose(Converter::eigenMatToCvMat(pose).inv());
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
        Vector3d point3d = v_p->estimate();
        landmarkIter->second.setPoint(Converter::vector3dToPoint3d(point3d));
    }
    
    // update nBundleAdjust
    nBundleAdjusted = int(views.size());
}



