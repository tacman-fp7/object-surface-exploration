/*
 * Author: Nawid Jamali
 * Project: TACMAN
 */

//#include <yarp/os/Time.h>
#include <iostream>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <exploreObject.h>
//#include <surfaceModelGP.h>
//#include <yarp/os/Time.h>
//#include <gurls++/gmat2d.h>
//#include "hand.h" //Testing only
//#include "objectModelGrid.h"

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
  

  /*objectExploration::ObjectModelGrid objectModel("testObject");
  yarp::sig::Vector startingPoint, endingPoint, nextSamplingPoint;
  startingPoint.resize(3);
  endingPoint.resize(3);
  startingPoint[0] = 0;
  startingPoint[1] = -1;
  startingPoint[2] = 0;
  endingPoint[0] = -1;
  endingPoint[1] = 0;
  endingPoint[2] = 0;
  objectModel.init(startingPoint, endingPoint);

  while(objectModel.getNextSamplingPos(nextSamplingPoint))
    objectModel.addContactPoint(nextSamplingPoint);

  objectModel.saveContactPoints();
*/


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
