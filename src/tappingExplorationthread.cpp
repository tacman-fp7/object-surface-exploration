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
    enum State{
        UNDEFINED,
        APPROACHING,
        INCONTACT,
        MOVELOCATION
    };

    State contactState = UNDEFINED;



    while(!isStopping()) // Keep running this
    {




        cout << "Tapping away!" << endl;

        /// Position the hand at the waypoint
        Vector px, ox;
        px.resize(3);
        px.zero();
        ox.resize(4);
        ox.zero();

        cout << "Getting the next waypoint...";
        if(_objectFeatures->getWayPoint(px, ox))
        {
            cout << "done!" << endl;

            contactState = APPROACHING;

            cout << "Moving to the waypoint...";
            // Go to the wayPoint if only it is a valid wayPoint.
            if(_robotCartesianController->goToPoseSync(px, ox))
                _robotCartesianController->waitMotionDone(0.1, 2);
            cout << "done!" << endl;
        }
        else
        {
            cout << "the Waypoint is invalid." << endl;
            if(contactState != UNDEFINED)
                contactState = MOVECONTACT;

        }

        cout << "Approaching...";
        /// Tell the finger controller to approach the object
        _objectFeatures->writeToFingerController("task add appr");
        _objectFeatures->writeToFingerController("start");


        cout << "Waiting for contact ...";

        // Wait for a contact or the distal angle being beyond certain limit
        bool inContact = true;
        while(_objectFeatures->getContactForce() < 3)
        {
            // TODO: implement a timeout
            if(_objectFeatures->getProximalJointAngle() > 50 || isStopping())
            {
                cout << "No contact detected." << endl;

                inContact = false;
                break;
            }


        }


        // Stop the approach
        _objectFeatures->writeToFingerController("stop");



        if(inContact)  // Check if we are in contact with the object
        {
            /////////////////////////////////////////////////////////////////////////
            /////////////////////////// We Have Contact /////////////////////////////
            /////////////////////////////////////////////////////////////////////////
            cout << "We have contact!" << endl;

            contactState = INCONTACT;

            _objectFeatures->writeToFingerController("task add ctrl 20");  // TODO: put it in the config file
            _objectFeatures->writeToFingerController("start");
        }
        else
        {

            ////////////////////////////////////////////////////////////////////////
            ////////////////////////// No contact //////////////////////////////////
            ////////////////////////////////////////////////////////////////////////

            // Stop the finger controller
            cout << "Stopping the finger controller...";
            _objectFeatures->writeToFingerController("stop");
            cout << "Done!" << endl;

            // Get the finger postion
            cout << "Reading the fingertip position...";
            _objectFeatures->getFingertipPose(finger_pos, finger_orient);
            cout << "Done!" << endl;
            cout << "Fingertip position: " << finger_pos.toString() << endl;

            yarp::os::Time::delay(1);

            cout << "Opening the hand...";
            //Open the finger
            _objectFeatures->openHand();

            //Wait for the fingertip to open
            while(!_objectFeatures->checkOpenHandDone() && !isStopping())
                ;
            cout << "Done!" << endl;




            // Calculate new waypoint, I should also check whether the robot has reached the limit,

            if(px[0] != 0) // This happens only when the waypoint is invalid
            {
                px[2] += finger_pos[2];
                _objectFeatures->setWayPoint(px, ox);

            }
            else
            {
                std::cerr << endl << "Got invalid waypoint" << endl;
            }




        }











    }


    // Make sure nothing is conrolling the finger
    _objectFeatures->writeToFingerController("stop");
    // Open the hand
    _objectFeatures->openHand();



}


bool TappingExplorationThread::threadInit()
{
    return true;
}


void TappingExplorationThread::threadRelease()
{

}

} // namespace objectExploration
