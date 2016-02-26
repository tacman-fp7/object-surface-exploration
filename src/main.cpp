/*
 * Author: Nawid Jamali
 * Project: TACMAN
 */

//#include <yarp/os/Time.h>
#include <iostream>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <exploreObject.h>
#include <surfaceModelGP.h>

using namespace yarp::os;
using namespace std;

int main(int argc, char *argv[])
{	
  Network yarp;
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

  objectExploration::SurfaceModelGP surfModel("hut");

  surfModel.loadContactData();
  surfModel.trainModel();
  surfModel.saveMeshCSV();
  surfModel.saveContactPoints("blindSearh");

  return 1;


  // Create a robot control server
  objectExploration::ExploreObject objectExplorationModule(rf);
  
  if(!objectExplorationModule.configure(rf))
    return 1;
 
  return objectExplorationModule.runModule();
  
  
}
