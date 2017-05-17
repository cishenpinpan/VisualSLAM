//
//  Frame.h
//  VisualSLAM
//
//  Created by Rong Yuan on 2/24/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#ifndef View_h
#define View_h

#include "Feature.h"
#include "Factory.h"
#include "Utility.h"
#include "CameraParameters.h"

namespace blindfind
{
    class View
    {
    public:
        View(const vector<Mat> _imgs, int _time);
        FeatureSet& getLeftFeatureSet();
        FeatureSet& getRightFeatureSet();
        long getId();
        vector<Mat> getImgs();
        void setImgs(const vector<Mat> _imgs);
        void deleteImgs();
        void setLeftFeatures(const FeatureSet _leftFeatureSet);
        void setRightFeatures(const FeatureSet _rightFeatureSet);
        Mat getPose();
        void setPose(const Mat _pose);
        void setPose(const Mat _R, const Mat _T);
        Mat getR();
        Mat getT();
        void setKeyView(){keyView = true;}
        void unsetKeyView(){keyView = false;}
        bool isKeyView(){return keyView;}
        bool isStereo(){return stereo;}
        int getTime(){return time;}
        void setGt(const Mat _gt){gt = _gt.clone();}
        Mat getGt(){return gt.clone();}
    private:
        long id;
        bool keyView;
        int time;
        vector<Mat> imgs;
        FeatureSet leftFeatureSet;
        FeatureSet rightFeatureSet;
        Mat pose;
        Mat gt;
        bool stereo;
    };
}

#endif /* View_h */
