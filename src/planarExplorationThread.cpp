#include <planarExplorationThread.h>
#include <iostream>
#include <yarp/sig/Vector.h>

using std::cout;
using std::endl;
using yarp::sig::Vector;

void objectExploration::PlanarExplorationThread::run()
{
  //cout << "Running the PlanarExplorationThread" << endl;
  Vector px, ox;
  _objectFeatures->getWayPoint(px, ox);
  _robotCartesianController->goToPoseSync(px, ox);
}


bool objectExploration::PlanarExplorationThread::threadInit()
{
  return true;
}


void objectExploration::PlanarExplorationThread::threadRelease()
{
  
}

