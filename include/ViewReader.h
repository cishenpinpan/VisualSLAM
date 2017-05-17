//
//  ImageReader.h
//  VisualSLAM
//
//  Created by Rong Yuan on 2/22/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#ifndef ViewReader_h
#define ViewReader_h

#include "View.h"

namespace blindfind
{
    class ViewReader
    {
    public:
        ViewReader(string _dataset, string _track, bool _stereo);
        // get next view
        vector<Mat> next();
        string getTrack(){return track;}
        
    private:
        string dataset;
        string track;
        bool stereo;
        int index;
        string left_right;
    };
}

#endif /* ViewReader_h */
