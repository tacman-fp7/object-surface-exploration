#include "tappingExplorationThread.h"
#include <iostream>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>
#include <cmath>
#include <ctime>
#include <yarp/os/Value.h>

#define DEBUG_LEVEL 1
#define FORCE_TH 0.8
namespace objectExploration
{



using std::cout;
using std::endl;
using std::cerr;

using yarp::sig::Vector;
using yarp::os::Value;


void TappingExplorationThread::run()
{

    Vector finger_pos, finger_orient;
    finger_pos.resize(3);
    finger_orient.resize(4);

    //State contactState = State::UNDEFINED;

    _contactState = APPROACH_OBJECT;
    _repeats = 0;
    _curDistal = 0;
    _curProximal = 10;

    _forceThreshold = FORCE_TH;
    while(!isStopping() && !(_contactState == STOP)) // Keep running this
    {


        _objectFeatures->updateContactState(_contactState);


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
    //_objectFeatures->writeToFingerController("stop");

    yarp::os::Time::delay(1);


}

void TappingExplorationThread::finshExploration()
{
    Vector starting_pos, starting_orient;
    starting_pos.resize(3);
    starting_orient.resize(4);

    _nGrid = 0;
    // Open the hand
    _objectFeatures->prepHand();
    _curProximal = 10;
    _curDistal = 70;
    Bottle msg;
    msg.clear();
    msg.addDouble(_curProximal);
    msg.addDouble(_curDistal);
    _objectFeatures->publishFingertipControl(msg);
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

    // Maintain the location for a given time period in the form of dealy
    yarp::os::Time::delay(0.01);

    // This is where I should log the position

    //_objectFeatures->writeToFingerController("stop");

    Vector starting_pos, starting_orient;
    starting_pos.resize(3);
    starting_orient.resize(4);
    _objectFeatures->getWayPoint(starting_pos, starting_orient, false);

    // Only lift it for the first repeat
    if(_repeats < 2)
        starting_pos[2] += 0.004;
    cout << "moving up...";
    _robotCartesianController->goToPoseSync(starting_pos,starting_orient);
    _robotCartesianController->waitMotionDone(0.1, 20);

    // Command the fingertip to move
    _curProximal = 10;
    _curDistal = 70;
    logFingertipControl();
    _objectFeatures->prepHand();
    //Wait for the fingertip to get to position
    while(!_objectFeatures->checkOpenHandDone() && !isStopping())
        ;
    cout << "done!" << endl;

    if(_repeats < 0) // TODO: change
    {

        _forceThreshold = 0.7 * FORCE_TH;
        _objectFeatures->setWayPoint(starting_pos, starting_orient); // Let the approach start from the new point
        _contactState = APPROACH_OBJECT;
        _repeats++;
    }
    else
    {
        _forceThreshold = FORCE_TH;
        _objectFeatures->getWayPoint(starting_pos, starting_orient);
        _contactState = MOVE_LOCATION;
        _repeats = 0;
    }

}

//void TappingExplorationThread::maintainContact()
//{

//    //////////////////////////////////////////// Straighten the finger ////////////////////////

//    // Get the current position of the fingertip
//    Vector contactPosition;
//   _objectFeatures->getIndexFingertipPosition(contactPosition);


//   // Open the fingertip


//   _objectFeatures->openIndexFinger();
//   while(!_objectFeatures->checkOpenHandDone() && !isStopping())
//       ;




//   // Read the current position
//    Vector openFingerPosition;
//    _objectFeatures->getIndexFingertipPosition(openFingerPosition);


//    // Move the hand to the new location.

//    Vector starting_pos, starting_orient;
//    starting_pos.resize(3);
//    starting_orient.resize(4);

//    _objectFeatures->getWayPoint(starting_pos, starting_orient);

//    starting_pos[0] += (contactPosition[0] - openFingerPosition[0]);

//    //cout << "B: " << starting_pos.toString() << endl;

//    _robotCartesianController->goToPoseSync(starting_pos, starting_orient);

//    _robotCartesianController->waitMotionDone(0.1, 10);


//    starting_pos[2] += ( contactPosition[2] - openFingerPosition[2])+ 0.008;

//    _robotCartesianController->goToPoseSync(starting_pos, starting_orient);

//    _robotCartesianController->waitMotionDone(0.1, 10);


//    //cout << "Delay" << endl;
//    //yarp::os::Time::delay(2);
//     _objectFeatures->fingerMovePosition(11, 15);
//     yarp::os::Time::delay(2);
// ///// End of finger ....

//    _objectFeatures->writeToFingerController("task add ctrl 10");
//    //_objectFeatures->writeToFingerController("task add appr");// TODO: put it in the config file
//    _objectFeatures->writeToFingerController("start");
//    yarp::os::Time::delay(2); //TODO: config file or some other type of criterion

//    /*
//    _objectFeatures->fingerMovePosition(7, 10, 100);
//     yarp::os::Time::delay(2);
//    _objectFeatures->fingerMovePosition(7,0, 100);
//     yarp::os::Time::delay(2);
//    _objectFeatures->fingerMovePosition(7, 5, 100);
//     yarp::os::Time::delay(2);
//    _objectFeatures->fingerMovePosition(7,5,10);
//*/
//    // Stop the maintain contact task
//    _objectFeatures->writeToFingerController("stop");
//    yarp::os::Time::delay(1);



//    // Move the hand up
//    starting_pos[2] += 0.005;

//    _robotCartesianController->goToPoseSync(starting_pos,starting_orient);
//    _robotCartesianController->waitMotionDone(0.1, 20);

//    //_objectFeatures->changeOrient(-0.3);
//    _robotCartesianController->waitMotionDone(0.1, 20);
//    //_objectFeatures->prepHand();


//    //Wait for the hand to go to open positon
//   // while(!_objectFeatures->checkOpenHandDone() && !isStopping())
//    //    ;

//    _contactState = MOVE_LOCATION;
//}


/**
 * @brief TappingExplorationThread::calculateNewWaypoint
 * If the
 */
void TappingExplorationThread::calculateNewWaypoint()
{
    // This is used to store the fingertip position when it is extended
    Vector fingertipPosition;
    fingertipPosition.resize(3);
    fingertipPosition.zero();

    // This is used for the final waypoint
    Vector waypointPos, waypointOrient;
    waypointPos.resize(3);
    waypointOrient.resize(4);

    // The check is necessary so we do not break the robot.
    if(_objectFeatures->getWayPoint(waypointPos, waypointOrient))
    {
        // Stop the finger controller
        // _objectFeatures->writeToFingerController("stop");

        // Get the finger postion
        _objectFeatures->getIndexFingertipPosition(fingertipPosition, _indexFingerEncoders);

        //Open the finger
        // TODO: make the logging less error prone.
        _curProximal = 10;
        _curDistal = 70;
        logFingertipControl();
        _objectFeatures->prepHand();
        //Wait for the fingertip to open
        while(!_objectFeatures->checkOpenHandDone() && !isStopping())
            ;

        yarp::os::Time::delay(0.01); // Why!?

        Vector prepDeltaPosition;
        _objectFeatures->getIndexFingertipPosition(prepDeltaPosition);

        fingertipPosition[2] -= prepDeltaPosition[2] + 0.003; // Take the current delta z out

        if(fingertipPosition[2] > 0.04)
        {
            cerr << "Warning! Delta z too big, capping it at 0.04";
            fingertipPosition[2] = 0.04;
        }

        waypointPos[2] += fingertipPosition[2];
        _objectFeatures->setWayPoint(waypointPos, waypointOrient);
        _contactState = APPROACH_OBJECT;
    }
    else
    {
        std::cerr << endl << "Got invalid waypoint" << endl;
    }
}

void TappingExplorationThread::logFingertipControl()
{
    Bottle msg;
    msg.clear();
    msg.addDouble(_curProximal);
    msg.addDouble(_curDistal);
    _objectFeatures->publishFingertipControl(msg);
}

void TappingExplorationThread::approachObject()
{

    // Position the hand at the waypoint
    Vector px, ox;
    px.resize(3);
    ox.resize(4);

    if(_objectFeatures->getWayPoint(px, ox, false))
    {
        // Go to the wayPoint if only it is a valid wayPoint.
        if(_robotCartesianController->goToPoseSync(px, ox))
            _robotCartesianController->waitMotionDone(0.1, 20);

        // Put the fingers in the right position
        _objectFeatures->prepHand();
        // TODO: fix the logging, this approach is error prone
        _curProximal = 10;
        _curDistal = 70;
        logFingertipControl();

        while(!_objectFeatures->checkOpenHandDone() && !isStopping())
            ;

    }else if(_contactState != UNDEFINED)
    {
        // If the contact point is not valid, then it is time to move to a new location
        // TODO: is this intuitive?
        _contactState = MOVE_LOCATION;

    }

    if(_contactState == APPROACH_OBJECT)
    {
        /// Tell the finger controller to approach the object
        _curProximal = 25;
        logFingertipControl();
        _objectFeatures->fingerMovePosition(11, _curProximal, 30);



        // _objectFeatures->writeToFingerController("task add appr");
        // _objectFeatures->writeToFingerController("start");


        std::clock_t time = std::clock();
        while((_objectFeatures->getContactForce()) < _forceThreshold) // TODO: Write a proper contact detctor
        {
            if(isStopping())
            {
                break;
            }
            else if(_objectFeatures->getProximalJointAngle() > _curDistal * 0.8) // stop if more than 80%
            {
                cout << "No contact detected." << endl;
                _contactState = CALCULATE_NEWWAYPONT;
                break;
            }
            else if( (std::clock() - time) / (double)(CLOCKS_PER_SEC) > 2)
            {
                cout << "No contact was detected -- timed out" << endl;
                _contactState = CALCULATE_NEWWAYPONT;
                break;
            }
        }

        // This is used later to move the relative to the contact
        // and prep position.
        _objectFeatures->getIndexFingerEncoder(_indexFingerEncoders);

        if(_contactState != CALCULATE_NEWWAYPONT)
        {
            cout << "We have detected contact" << endl;
            _contactState = MAINTAIN_CONTACT;
        }
        else if(_contactState == CALCULATE_NEWWAYPONT)
        {
            // No contact detected put the hand in prep position
            _objectFeatures->prepHand();
            _curProximal = 10;
            _curDistal = 70;
            logFingertipControl();

            // Wait for the hand to open;
            while(!_objectFeatures->checkOpenHandDone() && !isStopping())
                ;
        }
    }

    if(_contactState == MAINTAIN_CONTACT)
    {
        yarp::os::Time::delay(0.01);
        if(_objectFeatures->getContactForce() < _forceThreshold)
            _contactState == APPROACH_OBJECT;
    }
    //_objectFeatures->writeToFingerController("stop");

    return;
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

    double factor = 0.2;
    wayPoint_pos[1] += ((end_pos[1] - starting_pos[1]) * factor);
    wayPoint_pos[2] = starting_pos[2];

    _objectFeatures->setWayPoint(wayPoint_pos, wayPoint_orient);


    //cout << "WayPoint: " << fabs(wayPoint_pos[1] - end_pos[1]) << " " << "EndPos: " << fabs(end_pos[1] - starting_pos[1]) * factor << endl;
    if((fabs(wayPoint_pos[1] - starting_pos[1]) <= fabs(end_pos[1] - starting_pos[1]) ))
        _contactState = APPROACH_OBJECT;
    else
    {
        // I have to go to the next step of the grip

        if (_nGrid < 8)
        {
            cout << "Next grid" << endl;
            starting_pos[0] -= 0.01;
            end_pos[0] -= 0.01;

            _objectFeatures->setStartingPose(starting_pos, starting_orient);
            _objectFeatures->setEndPose(end_pos, end_orient);
            _nGrid++;
            _objectFeatures->setWayPoint(starting_pos, starting_orient);
            _contactState = APPROACH_OBJECT;
        }
        else
        {

#if DEBUG_LEVEL>=2
            cout << "State set to finished" << endl;
#endif
            _contactState = FINISHED;
        }
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
