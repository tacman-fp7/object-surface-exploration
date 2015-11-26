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

    while(true)
    {

        if(isStopping())
            break;

    cout << "Tapping away!" << endl;

    /// Position the hand at the waypoint
    Vector px, ox;
    if(_objectFeatures->getWayPoint(px, ox))
    {
        // Go to the wayPoint if only it is a valid wayPoint.
        _robotCartesianController->goToPoseSync(px, ox);
        _robotCartesianController->waitMotionDone();
    }

    /// Tell the finger controller to approach the object

    //_objectFeatures->writeToFingerController("task add open");
    /*   _objectFeatures->writeToFingerController("task add appr");
    _objectFeatures->writeToFingerController("task add ctrl 20");
    _objectFeatures->writeToFingerController("start");
*/

    //yarp::os::Time::delay(10);

    // _objectFeatures->writeToFingerController("open");


    _objectFeatures->writeToFingerController("task add appr");
    _objectFeatures->writeToFingerController("start");




    // Wait for a contact or the distal angle being beyond certain limit
    bool inContact = true;
 /*   while(_objectFeatures->getContactForce() < 1)
    {
        if(_objectFeatures->getProximalJointAngle() > 60)
        {
            inContact = false;
            break;
        }
    }
*/


    // Check if we are in contact with the object
    if(inContact)
    {
        cout << "We have contact!" << endl;

        _objectFeatures->writeToFingerController("task add ctrl 20");
        _objectFeatures->writeToFingerController("start");
    }



    // Move the finger to open postion

    // Move the hand to the waypoint in the begenning of the run

    // Decide what should be the next waypoint

    //while(true)
    //    ;

    yarp::os::Time::delay(3);

    _objectFeatures->writeToFingerController("stop");


}

}


bool TappingExplorationThread::threadInit()
{
    return true;
}


void TappingExplorationThread::threadRelease()
{

}

} // namespace objectExploration
