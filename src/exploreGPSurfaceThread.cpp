#include <exploreGPSurfaceThread.h>

#define FORCE_TH 1.6

namespace objectExploration
{

using yarp::sig::Vector;
using std::cout;
using std::endl;

void ExploreGPSurfaceThread::run()
{

    // Assuming that there is a valid model saved
    // in the working directory
    Vector startingPos, endingPos, startingOrient, endingOrient;
    _objectFeatures->getStartingPose(startingPos, startingOrient);
    _objectFeatures->getEndingPose(endingPos, endingOrient);
    initialiseGP(startingPos, startingOrient, endingPos, endingOrient);


}

bool ExploreGPSurfaceThread::initialiseGP(yarp::sig::Vector startingPos, yarp::sig::Vector startingOrient, yarp::sig::Vector endingPos, yarp::sig::Vector endingOrient)
{
    //double xMin, xMax, yMin, yMax, zMin;
    int nSteps = 20;


    _surfaceModel->loadContactData("GP");
    _surfaceModel->setBoundingBox(nSteps, 0/1000);
    _surfaceModel->trainModel();
    _surfaceModel->updateSurfaceEstimate();

}


} // end of namespace
