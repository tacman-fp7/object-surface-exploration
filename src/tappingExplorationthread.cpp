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

    Vector finger_pos, finger_orient;
    finger_pos.resize(3);
    finger_orient.resize(4);

    while(!isStopping()) // Keep running this
    {

      //  if(isStopping())
     //   {
            // Make sure we are not controlling the fingertip
   //         _objectFeatures->writeToFingerController("open");
   //         break;
   //     }
        cout << "Tapping away!" << endl;

        /// Position the hand at the waypoint
        Vector px, ox;
        if(_objectFeatures->getWayPoint(px, ox))
        {
            // Go to the wayPoint if only it is a valid wayPoint.
            if(_robotCartesianController->goToPoseSync(px, ox))
                _robotCartesianController->waitMotionDone();
        }

        cout << "Approaching..." << endl;
        /// Tell the finger controller to approach the object
        _objectFeatures->writeToFingerController("task add appr");
        _objectFeatures->writeToFingerController("start");




        // Wait for a contact or the distal angle being beyond certain limit
        bool inContact = true;
        while(_objectFeatures->getContactForce() < 3)
        {
            if(_objectFeatures->getProximalJointAngle() > 60 || isStopping())
            {
                cout << "No contact detected" << endl;

                inContact = false;
                break;
            }


        }




        if(inContact)  // Check if we are in contact with the object
        {
            cout << "We have contact!" << endl;

            _objectFeatures->writeToFingerController("task add ctrl 20");
            _objectFeatures->writeToFingerController("start");
        }
        else
        {

            // Get the finger postion
            _objectFeatures->getFingertipPose(finger_pos, finger_orient);

            //Open the finger
             _objectFeatures->setProximalAngle(0);

             //Wait until it is greater than 10 degress
             while(_objectFeatures->getProximalJointAngle() > 10)
                 ;

             // Move lower the hand

             cout << "Finger position: " << finger_pos.toString() << endl;

             px[2] = finger_pos[2];

             _objectFeatures->setWayPoint(px, ox);

             continue;

        }



        // Move the finger to open postion

        // Move the hand to the waypoint in the begenning of the run

        // Decide what should be the next waypoint

        //while(true)
        //    ;

        yarp::os::Time::delay(3);

        _objectFeatures->setProximalAngle(0);
        yarp::os::Time::delay(3);


    }

    //_objectFeatures->writeToFingerController("open");
}


bool TappingExplorationThread::threadInit()
{
    return true;
}


void TappingExplorationThread::threadRelease()
{

}

} // namespace objectExploration
