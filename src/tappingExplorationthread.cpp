#include "tappingExplorationthread.h"
#include <iostream>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>
#include <cmath>
#include <ctime>

#define DEBUG_LEVEL 1

namespace objectExploration
{



using std::cout;
using std::endl;
using std::cerr;

using yarp::sig::Vector;



void TappingExplorationThread::run()
{

    Vector finger_pos, finger_orient;
    finger_pos.resize(3);
    finger_orient.resize(4);

    //State contactState = State::UNDEFINED;

    _contactState = APPROACH_OBJECT;


    while(!isStopping() && !(_contactState == STOP)) // Keep running this
    {







        switch (_contactState)
        {

        case UNDEFINED:
            // This is the first round no approach has been made
            // Get the waypoint and set the state to approaching

            break;
        case  APPROACH_OBJECT:
            // Aproach and wait for contact
            // If contact set the state to contact
            // If reach the limit set the state to MOVELOCATION
#if DEBUG_LEVEL>=1
            cout << "Contact state is: approach" << endl;
#endif
            approachObject();
            break;
        case CALCULATE_NEWWAYPONT:
#if DEBUG_LEVEL>=1
            cout << "Contact state is: new waypoint" << endl;
#endif
            calculateNewWaypoint();
            break;
        case MAINTAIN_CONTACT:
#if DEBUG_LEVEL>=1
            cout << "Contact state is: maintain contact" << endl;
#endif
            // Maintain contact for a couple of seconds
            // Then set the state to move location
            maintainContact();
            break;
        case MOVE_LOCATION:
#if DEBUG_LEVEL>=1
            cout << "Contact state is: move location" << endl;
#endif
            // Calculate the next waypoint
            moveToNewLocation();
            //continue; // Just for now, before I clean the code
            break;
        case FINISHED:
#if DEBUG_LEVEL>=1
            cout << "Contact state is: finished" << endl;
#endif
            // I have to implement exit the thread procedure here
            finshExploration();
            break;
        }
    }

    // Make sure nothing is conrolling the finger
    _objectFeatures->writeToFingerController("stop");

    yarp::os::Time::delay(1);


}

void TappingExplorationThread::finshExploration()
{
    Vector starting_pos, starting_orient;
    starting_pos.resize(3);
    starting_orient.resize(4);

    // Open the hand
    _objectFeatures->prepHand();

    // Wait for the hand to open;
    while(!_objectFeatures->checkOpenHandDone() && !isStopping())
        ;


    // Lift the finger up
    // Get the current pose of the arm
    if(_robotCartesianController->getPose(starting_pos, starting_orient))
    {
        starting_pos[2] = 0.05; // TODO: remove the magic number
        _objectFeatures->setWayPoint(starting_pos, starting_orient);

        _objectFeatures->getWayPoint(starting_pos, starting_orient, false);

        _robotCartesianController->goToPoseSync(starting_pos, starting_orient);
        _robotCartesianController->waitMotionDone(0.1, 20);

    }




    if(_objectFeatures->getStartingPose(starting_pos, starting_orient))
    {
        //
        _robotCartesianController->goToPoseSync(starting_pos, starting_orient);
        _robotCartesianController->waitMotionDone(0.1, 20);
    }

    _contactState = STOP;

}

void TappingExplorationThread::maintainContact()
{

    //////////////////////////////////////////// Straighten the finger ////////////////////////

    // Get the current position of the fingertip
    Vector contactPosition;
   _objectFeatures->getIndexFingertipPosition(contactPosition);

   // Open the fingertip
   _objectFeatures->openIndexFinger();

   while(!_objectFeatures->checkOpenHandDone() && !isStopping())
       ;

   // Read the current position
    Vector openFingerPosition;
    _objectFeatures->getIndexFingertipPosition(openFingerPosition);


    // Move the hand to the new location.

    Vector starting_pos, starting_orient;
    starting_pos.resize(3);
    starting_orient.resize(4);

    _objectFeatures->getWayPoint(starting_pos, starting_orient);
    cout << "A: " << starting_pos.toString() << endl;

    starting_pos[0] -= contactPosition[0] - openFingerPosition[0];
    starting_pos[2] -= ( contactPosition[2] - openFingerPosition[2]);

    cout << "B: " << starting_pos.toString() << endl;

    _robotCartesianController->goToPoseSync(starting_pos, starting_orient);

    _robotCartesianController->waitMotionDone(0.1, 5);


    //cout << "Delay" << endl;
    //yarp::os::Time::delay(2);

 ///// End of finger ....

    _objectFeatures->writeToFingerController("task add ctrl 20");  // TODO: put it in the config file
    _objectFeatures->writeToFingerController("start");
    yarp::os::Time::delay(2); //TODO: config file or some other type of criterion

    _objectFeatures->fingerMovePosition(7, 10, 100);
     yarp::os::Time::delay(2);
    _objectFeatures->fingerMovePosition(7,0, 100);
     yarp::os::Time::delay(2);
    _objectFeatures->fingerMovePosition(7, 5, 100);
     yarp::os::Time::delay(2);
    _objectFeatures->fingerMovePosition(7,5,10);

    // Stop the maintain contact task
    _objectFeatures->writeToFingerController("stop");
    yarp::os::Time::delay(1);


    //_objectFeatures->prepHand();


    //Wait for the hand to go to open positon
    while(!_objectFeatures->checkOpenHandDone() && !isStopping())
        ;

    _contactState = MOVE_LOCATION;
}



void TappingExplorationThread::calculateNewWaypoint()
{

#if DEBUG_LEVEL>=2
    cout << "Calculating new waypoint" << endl;
#endif



    Vector fingertipPosition;
    fingertipPosition.resize(3);
    fingertipPosition.zero();

  /* Vector finger_pos, finger_orient;
    finger_pos.resize(3);
    finger_orient.resize(4);
    finger_pos.zero();
    finger_orient.zero();
*/
    Vector px, ox;
    px.resize(3);
    px.zero();
    ox.resize(4);
    ox.zero();


    if(_objectFeatures->getWayPoint(px, ox))
    {
        // Stop the finger controller
#if DEBUG_LEVEL>=2
        cout << "Stopping the finger controller...";
#endif



         _objectFeatures->writeToFingerController("stop");
         //_objectFeatures->setProximalAngle(_proximalAngle);
        // while(!_objectFeatures->checkOpenHandDone() && !isStopping())
        //     ;
        // yarp::os::Time::delay(1);




        //yarp::os::Time::delay(10);
#if DEBUG_LEVEL>=2
        cout << "Done!" << endl;
#endif


        // Get the finger postion
#if DEBUG_LEVEL>=2
        cout << "Reading the fingertip position...";
#endif
    _objectFeatures->getIndexFingertipPosition(fingertipPosition, _indexFingerEncoders);


#if DEBUG_LEVEL>=2
        cout << "Done!" << endl;
        cout << "Fingertip position: " << finger_pos.toString() << endl;
#endif



#if DEBUG_LEVEL>=2
        cout << "Opening the hand...";
#endif
        //Open the finger
        _objectFeatures->prepHand();

        //Wait for the fingertip to open
        while(!_objectFeatures->checkOpenHandDone() && !isStopping())
            ;

        //cout << "motion done!" << endl;
        yarp::os::Time::delay(1); //TODO: hack to make sure motion is done
        Vector prepDeltaPosition;
        _objectFeatures->getIndexFingertipPosition(prepDeltaPosition);

        fingertipPosition[2] -= prepDeltaPosition[2]; // Take the current delta z out

        //cout << "Delta z: " << fingertipZdisp << endl;
        if(fingertipPosition[2] > 0.02)
        {
            cerr << "Warning! Delta z too big, capping it at 0.02";
            fingertipPosition[2] = 0.02;
        }


#if DEBUG_LEVEL>=2
        cout << "Done!" << endl;
#endif



        px[2] -= fingertipPosition[2];
        _objectFeatures->setWayPoint(px, ox);

        _contactState = APPROACH_OBJECT;

    }
    else
    {
        std::cerr << endl << "Got invalid waypoint" << endl;
    }




}

void TappingExplorationThread::approachObject()
{


#if DEBUG_LEVEL>=2
    cout << "Approaching the object" << endl;
#endif

    /// Position the hand at the waypoint
    Vector px, ox;
    px.resize(3);
    px.zero();
    ox.resize(4);
    ox.zero();

    // Get the precontact force
    _preContactForce = _objectFeatures->getContactForce();

#if DEBUG_LEVEL>=2
    cout << "Getting the next waypoint...";
#endif
    if(_objectFeatures->getWayPoint(px, ox, false))
    {
#if DEBUG_LEVEL>=2
        cout << "done!" << endl;
        cout << "Moving to the waypoint...";
#endif
        // Go to the wayPoint if only it is a valid wayPoint.
        if(_robotCartesianController->goToPoseSync(px, ox))
            _robotCartesianController->waitMotionDone(0.1, 20);

        _objectFeatures->prepHand();

#if DEBUG_LEVEL>=2
        cout << "done!" << endl;
#endif
    }else if(_contactState != UNDEFINED)
    {
#if DEBUG_LEVEL>=2
        cout << "the Waypoint is invalid." << endl;
#endif
        _contactState = MOVE_LOCATION;
    }



    if(_contactState == APPROACH_OBJECT)
    {


#if DEBUG_LEVEL>=2
        cout << "Approaching...";
#endif
        /// Tell the finger controller to approach the object
        _objectFeatures->writeToFingerController("task add appr");
        _objectFeatures->writeToFingerController("start");
       // _objectFeatures->setProximalAngle(60);

#if DEBUG_LEVEL>=2
        cout << "Waiting for contact ...";
#endif

        // Wait for a contact or the distal angle being beyond certain limit
        // bool inContact = true;

        std::clock_t time = std::clock();
        while((_objectFeatures->getContactForce() - _preContactForce) < 2 ) // Write a proper contact detctor
        {

            //cout << "ContactForce: " << _objectFeatures->getContactForce() << endl;

            //_objectFeatures->adjustIndexFinger();

            if(isStopping())
            {
                break;
            }
            else if(_objectFeatures->getProximalJointAngle() > 20)
            {
#if DEBUG_LEVEL>=1
                cout << "No contact detected." << endl;
#endif
                _objectFeatures->getIndexFingerEncoder(_indexFingerEncoders);
                //_proximalAngle = 20;
                _contactState = CALCULATE_NEWWAYPONT;
                break;
            }
            else if( (std::clock() - time) / (double)(CLOCKS_PER_SEC) > 10)
            {
#if DEBUG_LEVEL>=1
                cout << "No contact was detected -- timed out" << endl;
#endif
                //_proximalAngle = _objectFeatures->getProximalJointAngle();
                _contactState = CALCULATE_NEWWAYPONT;
                break;
            }

        }

        if(_contactState != CALCULATE_NEWWAYPONT)
        {
#if DEBUG_LEVEL>=1
            cout << "We have detected contact" << endl;
#endif
            _contactState = MAINTAIN_CONTACT;
        }
        else if(_contactState == CALCULATE_NEWWAYPONT)
        {
            // No contact detected put the hand in prep position
            _objectFeatures->prepHand();
            // Wait for the hand to open;
            while(!_objectFeatures->checkOpenHandDone() && !isStopping())
                ;
        }
    }
_objectFeatures->writeToFingerController("stop");
}

void TappingExplorationThread::moveToNewLocation()
{

#if DEBUG_LEVEL>=2
    cout << "Moving to a new location" << endl;
#endif

    Vector starting_pos, starting_orient;
    Vector end_pos, end_orient;
    Vector wayPoint_pos, wayPoint_orient;

    starting_pos.resize(3);
    starting_orient.resize(4);

    end_pos.resize(3);
    end_orient.resize(4);

    wayPoint_pos.resize(3);
    wayPoint_orient.resize(4);

    if(!_objectFeatures->getStartingPose(starting_pos, starting_orient))
    {
        cerr  << "Cannot set a new location. Starting point is invalid" << endl;
        return;
    }

    if(!_objectFeatures->getDesiredEndPose(end_pos, end_orient))
    {
        cerr << "Cannot set a new location. Desired end-point is invalid" << endl;
        return;
    }


    // Get the current wayPoint
    if(_objectFeatures->getWayPoint(wayPoint_pos, wayPoint_orient, false))
    {
        cerr << "The current waypoint is invalid" << endl;
    }

    if(wayPoint_pos[0] == 0)
    {
        cerr << "Cannot set new location, previous waypoint is invalid" << endl;
    }


    ////////////////////////////////////////////////////////////////////////////////
    //////////////// Calculating a new waypoint for the travaersal /////////////////
    ////////////////////////////////////////////////////////////////////////////////

    wayPoint_pos[1] += ((end_pos[1] - starting_pos[1]) * 0.2);
    wayPoint_pos[2] = starting_pos[2];

    _objectFeatures->setWayPoint(wayPoint_pos, wayPoint_orient);


    //cout << "WayPoint: " << fabs(wayPoint_pos[1]) << " " << "EndPos: " << fabs(end_pos[1]) << endl;
    if(fabs(wayPoint_pos[1]) > fabs(end_pos[1]))
        _contactState = APPROACH_OBJECT;
    else
    {
#if DEBUG_LEVEL>=2
        cout << "State set to finished" << endl;
#endif
        _contactState = FINISHED;
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
