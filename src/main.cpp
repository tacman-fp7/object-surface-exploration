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


 /*   objectExploration::SurfaceModelGPActive surf("TestObject");
    surf.setBoundingBox(-10, 210, -10, 210,20);
    yarp::sig::Vector fingertipPosition;
    fingertipPosition.resize(3);

    for (int i = 0; i < 100; i += 3){
        fingertipPosition[0] = i;
        fingertipPosition[1] = i+1;
        fingertipPosition[2] = i+2;
        surf.addContactPoint(fingertipPosition, 0);
        surf.trainModel();
        surf.updateSurfaceEstimate();
    }

    for (int i = 101; i < 200; i += 3){
        fingertipPosition[0] = i;
        fingertipPosition[1] = i+1;
        fingertipPosition[2] = i+2;
        surf.addContactPoint(fingertipPosition, 1);
        surf.trainModel();
        surf.updateSurfaceEstimate();

    }

    return 1; */


 /*   objectExploration::SurfaceModelGPActive surfaceModel("hut");
    gurls::gMat2D<double> inputTraining, targetTraining;
    inputTraining.readCSV("myInputTraining.csv");
    targetTraining.readCSV("myTargetTraining.csv");

    double xMin = inputTraining.min(gurls::COLUMNWISE)->at(0);
    double xMax = inputTraining.max(gurls::COLUMNWISE)->at(0);
    double yMin = inputTraining.min(gurls::COLUMNWISE)->at(1);
    double yMax = inputTraining.max(gurls::COLUMNWISE)->at(1);
    double zMin = -0.15;
    int nSteps = 20;

    surfaceModel.padBoundingBox(xMin, xMax, yMin, yMax, zMin, nSteps, 0.0/1000);
    surfaceModel.setBoundingBox(nSteps, 0/1000);
    surfaceModel.trainModel();
    surfaceModel.updateSurfaceEstimate();

    for (int i = 80; i < inputTraining.rows(); i++){
        yarp::sig::Vector position;
        position.resize(3);
        position(0) = inputTraining(i,0);
        position(1) = inputTraining(i,1);
        position(3) = targetTraining(i,0);

        surfaceModel.addContactPoint(position);
        surfaceModel.trainModel();
        surfaceModel.updateSurfaceEstimate();

    }
    return 0;*/

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
