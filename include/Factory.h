//
//  Factory.hpp
//  VisualSLAM
//
//  Created by Rong Yuan on 3/8/17.
//  Copyright © 2017 Rong Yuan. All rights reserved.
//

#ifndef Factory_h
#define Factory_h

#include <stdio.h>
#include <vector>

using namespace std;

namespace blindfind
{
    // singleton
    class IdGenerator
    {
    private:
        long int nextId;
        static IdGenerator* instance;
        IdGenerator(){nextId = 0;};
    public:
        static IdGenerator* createInstance()
        {
            if(!instance)
                instance = new IdGenerator();
            return instance;
        }
        long int next(){return nextId++;}
        vector<long int> next(const int length)
        {
            int len = length;
            vector<long int> res;
            while(len--)
                res.push_back(next());
            return res;
        }
    };
}

#endif /* Factory_hpp */
