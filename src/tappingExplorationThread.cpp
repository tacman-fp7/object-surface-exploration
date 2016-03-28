#include "tappingExplorationThread.h"
#include <iostream>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>
#include <cmath>
#include <ctime>
#include <yarp/os/Value.h>

#define DEBUG_LEVEL 1
#define FORCE_TH 1.6
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
    _nRepeats = 0;
   // _curDistal = 0;
   // _curProximal = 10;
   // _curAbduction = 0;


    _forceThreshold = FORCE_TH;

    // put the finger in known position
    _curProximal = 0;
    _curAbduction = 0;
    moveIndexFingerBlocking(_curProximal, _curAbduction, 40);

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
    //_objectFeatures->prepHand();




 /*   _objectFeatures->setProximalAngle(_curProximal);
    Bottle msg;
    msg.clear();
    msg.addDouble(_curProximal);
    msg.addDouble(_curDistal);
    msg.addDouble(_curAbduction);
    _objectFeatures->publishFingertipControl(msg);
    // Wait for the hand to open;
    while(!_objectFeatures->checkOpenHandDone() && !isStopping())
        ;
*/

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


    // Move the fingertip
    _curProximal = 0;
    _curAbduction = 0;
    moveIndexFingerBlocking(_curProximal, _curAbduction, 40);


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

    _curProximal = 10;
    _curAbduction = 0;
    moveIndexFingerBlocking(_curProximal, _curAbduction, 40);

    if(_repeats < _nRepeats) // TODO: change
    {

        _forceThreshold = 0.7 * FORCE_TH;
        _contactState = APPROACH_OBJECT;
        _repeats++;
    }
    else
    {
        _forceThreshold = FORCE_TH;
        _contactState = MOVE_LOCATION;
        _repeats = 0;
    }

}



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


        // Get the finger postion
        _objectFeatures->getIndexFingertipPosition(fingertipPosition, _indexFingerEncoders);

        //Open the finger
        _curProximal = 10;
        moveIndexFingerBlocking(_curProximal, _curAbduction, 40);


        Vector prepDeltaPosition;
        _objectFeatures->getIndexFingertipPosition(prepDeltaPosition);

        fingertipPosition[2] -= prepDeltaPosition[2];// + 0.003; // Take the current delta z out

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

/*void TappingExplorationThread::logFingertipControl()
{
    Bottle msg;
    msg.clear();
    msg.addDouble(_curProximal);
    msg.addDouble(_curDistal);
    msg.addDouble(_curAbduction);
    _objectFeatures->publishFingertipControl(msg);
}
*/

void TappingExplorationThread::moveArmToWayPoint(yarp::sig::Vector pos, yarp::sig::Vector orient)
{
    Vector indexFingerAngles;
    if(_robotCartesianController->goToPoseSync(pos, orient))
    {
        bool motionDone = false;
        while(!motionDone)
        {
            if(_objectFeatures->getContactForce() > _forceThreshold)
            {
                cout  << "Abandoned motion due to force" << endl;
                _robotCartesianController->stopControl();

                break;
            }

            _objectFeatures->getIndexFingerAngles(indexFingerAngles);

            if(indexFingerAngles[1] < 20 )
            {
                cout << "Abandoned motion due to angles" << endl;
                _robotCartesianController->stopControl();
                cout << "Angles: " << indexFingerAngles.toString() << endl;

                break;
            }
            _robotCartesianController->checkMotionDone(&motionDone);
        }
    }
}

void TappingExplorationThread::moveIndexFingerBlocking(double proximalAngle, double distalAngle, double abductionAngle, double speed)
{
    _objectFeatures->setIndexFingerAngles(proximalAngle, distalAngle, abductionAngle, speed);
    while(!_objectFeatures->checkOpenHandDone() && !isStopping() )
        ;
}

void TappingExplorationThread::moveIndexFingerBlocking(double proximalAngle, double abductionAngle, double speed)
{
    moveIndexFinger(proximalAngle, abductionAngle);
    while(!_objectFeatures->checkOpenHandDone() && !isStopping() )
        ;
}

void TappingExplorationThread::moveIndexFinger(double proximalAngle, double abductionAngle, double speed)
{
    _curProximal = proximalAngle;
    _curAbduction = abductionAngle;
    //_curDistal = 90 - _curProximal;
    //logFingertipControl();
    //_objectFeatures->setProximalAngle(_curProximal);
    _objectFeatures->setIndexFingerAngles(_curProximal, _curAbduction, speed);

}

void TappingExplorationThread::approachObject()
{

    // Position the hand at the waypoint
    Vector px, ox;
    px.resize(3);
    ox.resize(4);
    //Vector indexFingerAngles;
    double maxProximal = 40;

    ///// Put the index finger in the starting position /////
    moveIndexFingerBlocking(10, _curAbduction, 40);


    // Go to the wayPoint if only it is a valid wayPoint.
    if(_objectFeatures->getWayPoint(px, ox, false))
    {
        moveArmToWayPoint(px, ox);
    }
    else if(_contactState != UNDEFINED)
    {
        // If the contact point is not valid, then it is time to move to a new location
        // TODO: is this intuitive?
        _contactState = MOVE_LOCATION;
    }

    if(_contactState == APPROACH_OBJECT)
    {
        // Tell the finger controller to approach the object
        moveIndexFinger(maxProximal, _curAbduction);

        // Wait for contact event
        detectContact(maxProximal);

        if(_contactState != CALCULATE_NEWWAYPONT)
        {
            cout << "We have detected contact" << endl;
            _contactState = MAINTAIN_CONTACT;
        }
        else if(_contactState == CALCULATE_NEWWAYPONT)
        {
            // No contact detected put the hand in prep position
            moveIndexFingerBlocking(10, _curAbduction, 40);
        }
    }

    // Confirm cotntact
    if(_contactState == MAINTAIN_CONTACT)
    {
        confrimContact(maxProximal);
    }
    return;
}

void TappingExplorationThread::detectContact(double maxAngle)
{
    Vector indexFingerAngles;
    std::clock_t time = std::clock();
    while((_objectFeatures->getContactForce()) < _forceThreshold)
    {
        // Get the angles
        _objectFeatures->getIndexFingerAngles(indexFingerAngles);


        if(isStopping() || (_contactState == STOP))
        {
            break;
        }
        else if(_objectFeatures->getProximalJointAngle() > maxAngle * 0.95)
        {
            cout << "No contact detected." << endl;
            _contactState = CALCULATE_NEWWAYPONT;
            break;
        }
        else if( (std::clock() - time) / (double)(CLOCKS_PER_SEC) > 1)
        {
            cout << "No contact was detected -- timed out" << endl;
            _contactState = CALCULATE_NEWWAYPONT;
            break;
        }
        else if(indexFingerAngles[1] < 3)
        {
            // We have contact without force
            cout << "No froce but angle exceeded the limit " << endl;
            _contactState = EXCEEDED_ANGLE;//MAINTAIN_CONTACT;
            break;
        }
    }

    // This is used later to move the relative to the contact
    // and prep position.
    _objectFeatures->getIndexFingerEncoder(_indexFingerEncoders);
}

bool TappingExplorationThread::confrimContact(double maxAngle)
{
    bool ret = false;
    cout << "Checking the contact...";
    moveIndexFingerBlocking(10/2, _curAbduction, 40);
    //cout << "Move done" << endl;
    //yarp::os::Time::delay(1);
    _contactState = APPROACH_OBJECT;

    Vector indexFingerAngles;
    double angle;
    for (int i = 10; i < maxAngle * 2; i++)
    {
        angle = i/2;
        moveIndexFinger(angle, _curAbduction);
        //cout << "Moving finger" << endl;

        while(!_objectFeatures->checkOpenHandDone())
        {
            _objectFeatures->getIndexFingerAngles(indexFingerAngles);

            if(_objectFeatures->getContactForce() >= _forceThreshold)
            {
                cout << "contact confirmed" << endl;
                _contactState = MAINTAIN_CONTACT;
                ret = true;
                break;
            }
            else if(indexFingerAngles[1] < 1)
            {
                cout << "...[exceeded angle]..." << endl;
                //_contactState = EXCEEDED_ANGLE;
                _contactState = MAINTAIN_CONTACT;
                ret = true;
                break;
            }
        }
        if(_contactState == MAINTAIN_CONTACT || _contactState == EXCEEDED_ANGLE)
            break;

    }

    if(_contactState == EXCEEDED_ANGLE)
    {
        // If contact cannot be confirmed by tactile sensor
        // But the angle exceeds, then add a little jitter to the
        // position and reapproach the point.
        double r = ((double) rand() / (RAND_MAX)) - 0.5;
        Vector pos, orient;
        _objectFeatures->getWayPoint(pos, orient, false);
        cout << "Pos: " << pos.toString() << endl;
        pos[0] += r * 0.05;
        pos[1] += r * 0.05;
        pos[2] += 0.001;
        cout << "Pos: " << pos.toString() << endl;

        _objectFeatures->setWayPoint(pos, orient);
        cout << "contact NOT confirmed" << endl;
        _contactState = APPROACH_OBJECT;
    }
    else if( _contactState == APPROACH_OBJECT)
    {

        _contactState = CALCULATE_NEWWAYPONT;
        cout << "contact NOT confirmed" << endl;
    }

    return ret;

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
