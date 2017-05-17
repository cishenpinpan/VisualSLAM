//
//  FeatureTracker.h
//  VisualSLAM
//
//  Created by Rong Yuan on 2/24/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#ifndef ViewTracker_h
#define ViewTracker_h
#include "View.h"
#include "Canvas.h"
#include "FeatureTracker.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
//#include "g2o/math_groups/se3quat.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"

using namespace Eigen;

namespace blindfind
{
    class ViewTracker
    {
    public:
        ViewTracker();
        ~ViewTracker();
        void addView(View *v);
        vector<View*> getViews();
        void bundleAdjust(int option, bool global);
        void computeLandmarks();
        void computeLandmarksStereo();
        void eraseLandmarks();
        map<long, Landmark>& getLandmarkBook(){return landmarkBook;}
        View* getLastView();
        View* getLastKeyView(){return keyViews.back();}
        vector<View*> getKeyViews(){return keyViews;}
        vector<View*> getLastTwoKeyViews(){return {keyViews[keyViews.size() - 2], keyViews.back()};}
        vector<View*> getLastNKeyViews(int N);
        View* popFirstView();
        View* popLastView();
        void setKeyView(View *v);
        void updateLandmarks(map<long, Landmark> &ref);
    private:
        vector<View*> views;
        vector<View*> keyViews;
        map<long, View*> viewBook;
        map<long, Landmark> landmarkBook;
    };
}


#endif /* ViewTracker_h */
