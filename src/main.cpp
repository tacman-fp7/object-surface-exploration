/*
 * Author: Nawid Jamali
 * Project: TACMAN
 */

//#include <yarp/os/Time.h>
#include <iostream>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <exploreObject.h>
#include <surfaceModelGPActive.h>
//#include <yarp/os/Time.h>
//#include <gurls++/gmat2d.h>
//#include "hand.h" //Testing only
//#include "objectModelGrid.h"
#include <yarp/sig/Vector.h>

#include <surfaceModelGPActive.h>
#include <gurls++/gmat2d.h>

//using namespace yarp::os;
//using namespace std;

using std::cout;
using std::endl;

int main(int argc, char *argv[])
{	
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {

        cout << "Could not contact the Yarp server, quitting." << endl;
        return false;
    }




    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("object-surface-exploration");
    rf.setDefaultConfigFile("objectExplorationConfig.ini");
    rf.configure(argc, argv);


    // Create a robot control server
    objectExploration::ExploreObject objectExplorationModule(rf);

    if(!objectExplorationModule.configure(rf))
        return 1;

    return objectExplorationModule.runModule();


}
