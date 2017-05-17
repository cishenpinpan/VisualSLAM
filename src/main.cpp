#include "Odometry.h"

#define DATASET "KITTI"
#define STEREO true
#define MONOCULAR false

using namespace blindfind;

int main( int argc, char** argv )
{
    // start monocular odometry
    vector<View*> views;
    Odometry *odometryHandler = new Odometry();
    
    // use the views above
    // triangulation
    odometryHandler->run(views, TRIANGULATION);
    
    return 0;
}


