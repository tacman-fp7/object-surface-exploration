#include <planarExplorationThread.h>
#include <iostream>
#include <yarp/sig/Vector.h>

namespace objectExploration
{


using std::cout;
using std::endl;
using yarp::sig::Vector;

void PlanarExplorationThread::run()
{
  //cout << "Running the PlanarExplorationThread" << endl;
  Vector px, ox;
  _objectFeatures->getWayPoint(px, ox);
  _robotCartesianController->goToPoseSync(px, ox);
}


bool PlanarExplorationThread::threadInit()
{
  return true;
}


void PlanarExplorationThread::threadRelease()
{
  
}

} // namespace objectExploration
