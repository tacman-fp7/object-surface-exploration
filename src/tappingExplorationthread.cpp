#include "tappingExplorationthread.h"
#include <iostream>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>

namespace objectExploration
{


using std::cout;
using std::endl;
using yarp::sig::Vector;

void TappingExplorationThread::run()
{

    cout << "Tapping away!" << endl;

    /// Position the hand at the waypoint
    Vector px, ox;
    _objectFeatures->getWayPoint(px, ox);
    _robotCartesianController->goToPoseSync(px, ox);
    _robotCartesianController->waitMotionDone();


    /// Tell the finger controller to approach the object

    //_objectFeatures->writeToFingerController("task add open");
    _objectFeatures->writeToFingerController("task add appr");
    _objectFeatures->writeToFingerController("task add ctrl 30");
    _objectFeatures->writeToFingerController("start");


     yarp::os::Time::delay(10);

     _objectFeatures->writeToFingerController("open");

    // If there is no contact move further down, esle
    // activate the maintain contact

    // Wait for a short period

    // Move the finger to open postion

    // Move the hand to the waypoint in the begenning of the run

    // Decide what should be the next waypoint

    //while(true)
    //    ;



}


bool TappingExplorationThread::threadInit()
{
    return true;
}


void TappingExplorationThread::threadRelease()
{

}

} // namespace objectExploration
