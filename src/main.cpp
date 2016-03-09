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
#include <yarp/os/Time.h>
#include <gurls++/gmat2d.h>

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


  /////////////////////////////////////
  /// \brief surfModel
  ///

 /* gurls::gMat2D <double> a;
  for(int i = 0; i < 10; i++){
  a.resize(a.rows() +1,2);
  a(a.rows() - 1, 0) = i;
  a(a.rows() - 1, 1) = i+10;
  }

  for(int i = 0; i < a.rows(); i++){
      a(i,0) = i; a(i,1) = i+10;
      cout << a(i,0) << ", " << a(i, 1) << endl;
  }

  return 0;

*/
          ////////
 // objectExploration::SurfaceModelGP surfModel("hut");

  //surfModel.loadContactData("blindSearch");

 /* for (int i = 0; i < 100; i++){
  surfModel.trainModel();
  surfModel.updateSurfaceEstimate();
  yarp::sig::Vector fingertipPosition;
  surfModel.getMaxVariancePose(fingertipPosition);
  cout << "Max var (main): " << fingertipPosition.toString();

  fingertipPosition[2] = 0;
  surfModel.addContactPoint(fingertipPosition);
  surfModel.saveContactPoints();
    yarp::os::Time::delay(1);
  }
  return 1;
  */

  // Create a robot control server
  objectExploration::ExploreObject objectExplorationModule(rf);
  
  if(!objectExplorationModule.configure(rf))
    return 1;
 
  return objectExplorationModule.runModule();
  
  
}
