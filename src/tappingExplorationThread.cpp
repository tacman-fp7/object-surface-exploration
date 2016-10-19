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



    _contactState = APPROACH_OBJECT;
    _repeats = 0;
    _nRepeats = 0;
    _forceThreshold = FORCE_TH;

    // put the finger in known position
    _curProximal = 0;
    _curAbduction = 0;
    moveExplorationFingerBlocking(_curProximal, _curAbduction, 40);

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



    yarp::os::Time::delay(1);  //TODO: Test if this is needed


}


TappingExplorationThread::TappingExplorationThread(int period, Hand* robotHand, Finger* explorationFinger,
                                                 Finger* auxiliaryFinger,  string objectName, ObjectFeaturesThread* objectFeatures):
    ExplorationStrategyThread(period, robotHand, explorationFinger, auxiliaryFinger, objectName,
                              objectFeatures){
    _nGrid = 0;
    _forceThreshold = FORCE_TH;
    _curAbduction = -10;
    _curAbduction = -10;
    _nRepeats = 0;

    _skinManagerCommand.open("/object-exploration/skinManager/rpc:o");
    yarp::os::Network::connect("/object-exploration/skinManager/rpc:o", "/skinManager/rpc");



    //_indexFingerEncoders.resize(3);
    //_indexFingerEncoders.zero();
}

void TappingExplorationThread::setNRepeats(int nRepeats){
    _nRepeats = nRepeats;
}


void TappingExplorationThread::finshExploration()
{
    Vector starting_pos, starting_orient;
    starting_pos.resize(3);
    starting_orient.resize(4);

    _nGrid = 0;


    // Lift the finger up
    // Get the current pose of the arm
    if( _robotHand->getPose(starting_pos, starting_orient))// _robotCartesianController->getPose(starting_pos, starting_orient))
    {
        starting_pos[2] = 0.05; // TODO: remove the magic number
        _robotHand->setWayPoint(starting_pos, starting_orient);

        _robotHand->getWayPoint(starting_pos, starting_orient, false);

        _robotHand->goToPoseSync(starting_pos, starting_orient, 20);
       // _robotCartesianController->goToPoseSync(starting_pos, starting_orient);
       // _robotCartesianController->waitMotionDone(0.1, 20);

    }


    // Move the fingertip
    _curProximal = 0;
    _curAbduction = 0;
    moveExplorationFingerBlocking(_curProximal, _curAbduction, 40);


    if(_robotHand->getStartingPose(starting_pos, starting_orient))
    {
        _robotHand->goToPoseSync(starting_pos, starting_orient, 20);
        //_robotCartesianController->goToPoseSync(starting_pos, starting_orient);
        //_robotCartesianController->waitMotionDone(0.1, 20);
    }

    _contactState = STOP;

}

void TappingExplorationThread::maintainContact()
{

    _curProximal = 10;
    _curAbduction = 0;
    moveExplorationFingerBlocking(_curProximal, _curAbduction, 40);

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
    if(_robotHand->getWayPoint(waypointPos, waypointOrient))
    {


        // Get the finger postion
        _explorationFinger->getPositionCorrected(fingertipPosition, _explorationFingerEncoders);

        //_explorationFinger->getPosition()
        //Open the finger
        _curProximal = 10;
        moveExplorationFingerBlocking(_curProximal, _curAbduction, 40);


        Vector prepDeltaPosition;
        //_robotHand->getIndexFingertipPosition(prepDeltaPosition);
        _explorationFinger->getPositionCorrected(prepDeltaPosition);
        fingertipPosition[2] -= prepDeltaPosition[2];// + 0.003; // Take the current delta z out

        if(fingertipPosition[2] > 0.04)
        {
            cerr << "Warning! Delta z too big, capping it at 0.04";
            fingertipPosition[2] = 0.04;
        }

        waypointPos[2] += fingertipPosition[2];
        _robotHand->setWayPoint(waypointPos, waypointOrient);
        _contactState = APPROACH_OBJECT;
    }
    else
    {
        std::cerr << endl << "Got invalid waypoint" << endl;
    }
}



void TappingExplorationThread::moveArmToWayPoint(yarp::sig::Vector pos, yarp::sig::Vector orient)
{
    Vector explorationFingerAngles;
    if( _robotHand->goToPoseSync(pos, orient))//_robotCartesianController->goToPoseSync(pos, orient))
    {
        bool motionDone = false;
        while(!motionDone)
        {
            if(_explorationFinger->getContactForce() > _forceThreshold)
            {
                cout  << "Abandoned motion due to force" << endl;
                //_robotCartesianController->stopControl();
                _robotHand->stopControl();
                break;
            }

            _explorationFinger->getAngels(explorationFingerAngles);

            //_robotHand->getIndexFingerAngles(indexFingerAngles);

            if(explorationFingerAngles[1] < 5 )
            {
                cout << "Abandoned motion due to angles" << endl;
                _robotHand->stopControl();
                //_robotCartesianController->stopControl();
                cout << "Angles: " << explorationFingerAngles.toString() << endl;

                break;
            }
            //_robotCartesianController->checkMotionDone(&motionDone);
            _robotHand->checkMotionDone(&motionDone);
        }
    }
}

void TappingExplorationThread::moveExplorationFingerBlocking(double proximalAngle, double distalAngle, double abductionAngle, double speed)
{

    _explorationFinger->setAngles(proximalAngle, distalAngle, speed);
    _robotHand->setAbduction(abductionAngle, speed);

    while(!_explorationFinger->checkMotionDone()){
        ;
    }

//TODO: should wait for abduction as well!


    //_robotHand->setIndexFingerAngles(proximalAngle, distalAngle, abductionAngle, speed);
//    while(!_robotHand->checkOpenHandDone() && !isStopping() )
//        ;
}

void TappingExplorationThread::moveExplorationFingerBlocking(double proximalAngle, double abductionAngle, double speed)
{
    moveExplorationFinger(proximalAngle, abductionAngle);
    while(!_explorationFinger->checkMotionDone() && !isStopping() )
        ;
}

void TappingExplorationThread::moveExplorationFinger(double proximalAngle, double abductionAngle, double speed)
{
    _curProximal = proximalAngle;
    _curAbduction = abductionAngle;
    _robotHand->setAbduction(abductionAngle, speed);
    _explorationFinger->setAngles(_curProximal, speed);

}

void TappingExplorationThread::moveFinger(Finger *finger, double proximalAngle, double abductionAngle, double speed)
{
    //_curProximal = proximalAngle;
    //_curAbduction = abductionAngle;
    _robotHand->setAbduction(abductionAngle, speed);
    finger->setAngles(proximalAngle, speed);

}

void TappingExplorationThread::moveFingerBlocking(Finger *finger, double proximalAngle, double abductionAngle, double speed)
{
    moveFinger(finger, proximalAngle, abductionAngle);
    while(!finger->checkMotionDone() && !isStopping() )
        ;
}

void TappingExplorationThread::moveFingerBlocking(Finger *finger, double proximalAngle, double distalAngle, double abductionAngle, double speed)
{

    finger->setAngles(proximalAngle, distalAngle, speed);
    _robotHand->setAbduction(abductionAngle, speed);

    while(!finger->checkMotionDone()){
        ;
    }

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
    moveExplorationFingerBlocking(10, _curAbduction, 40);


    // Go to the wayPoint if only it is a valid wayPoint.
    if(_robotHand->getWayPoint(px, ox, false))
    {
        moveArmToWayPoint(px, ox);
    }
    else if(_contactState != UNDEFINED)
    {
        // If the contact point is not valid, then it is time to move to a new location
        // TODO: is this intuitive?
        _contactState = MOVE_LOCATION;
    }

    // Debugging
   //  moveIndexFingerBlocking(30, _curAbduction, 40);

   // _contactState = MAINTAIN_CONTACT;
   // return;
    ///////

    if(_contactState == APPROACH_OBJECT)
    {
        // Tell the finger controller to approach the object
        moveExplorationFinger(maxProximal, _curAbduction);

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
            moveExplorationFingerBlocking(10, _curAbduction, 40);
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
    Vector explorationFingerAngles;
    std::clock_t time = std::clock();
    while((_explorationFinger->getContactForce()) < _forceThreshold)
    {
        // Get the angles
        _explorationFinger->getAngels(explorationFingerAngles);

        //cout << "Finger angles: " << indexFingerAngles.toString() << endl;
        //_robotHand->getIndexFingerAngles(indexFingerAngles);


        if(isStopping() || (_contactState == STOP))
        {
            break;
        }
        else if(explorationFingerAngles[0] > maxAngle * 0.95) //Proximal angle
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
        else if(explorationFingerAngles[1] < 3) // Middle angle
        {
            // We have contact without force
            cout << "No froce but middle angle exceeded the limit " << endl;
            _contactState = EXCEEDED_ANGLE;//MAINTAIN_CONTACT;
            break;
        }
    }

    // This is used later to move the relative to the contact
    // and prep position.
    _explorationFinger->readEncoders(_explorationFingerEncoders);
    //_robotHand->getIndexFingerEncoder(_indexFingerEncoders);
}

bool TappingExplorationThread::confrimContact(double maxAngle)
{
    bool ret = false;
    cout << "Checking the contact...";
    moveExplorationFingerBlocking(10/2, _curAbduction, 40);
    //cout << "Move done" << endl;
    //yarp::os::Time::delay(1);
    _contactState = APPROACH_OBJECT;

    Vector indexFingerAngles;
    double angle;
    for (int i = 10; i < maxAngle * 2; i++)
    {
        angle = i/2;
        moveExplorationFinger(angle, _curAbduction);
        //cout << "Moving finger" << endl;

        while(!_explorationFinger->checkMotionDone())
        {
            _explorationFinger->getAngels(indexFingerAngles);

            //_robotHand->getIndexFingerAngles(indexFingerAngles);

            if(_explorationFinger->getContactForce() >= _forceThreshold)
            {
                cout << "contact confirmed" << endl;
                _contactState = MAINTAIN_CONTACT;
                ret = true;
                break;
            }
            else if(indexFingerAngles[1] < 1)
            {
                cout << "...[exceeded angle]..." << endl;
                // _contactState = EXCEEDED_ANGLE;
                // ret = false;
                _contactState = MAINTAIN_CONTACT; // We consider it a contact
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
        _robotHand->getWayPoint(pos, orient, false);
        cout << "Pos: " << pos.toString() << endl;
        pos[0] += r * 0.0125;
        pos[1] += r * 0.0125;
        pos[2] += 0.003;
        cout << "Pos: " << pos.toString() << endl;

        _robotHand->setWayPoint(pos, orient);
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

    if(!_robotHand->getStartingPose(starting_pos, starting_orient))
    {
        cerr  << "Cannot set a new location. Starting point is invalid" << endl;
        return;
    }

    if(!_robotHand->getEndPose(end_pos, end_orient))
    {
        cerr << "Cannot set a new location. Desired end-point is invalid" << endl;
        return;
    }


    // Get the current wayPoint
    if(_robotHand->getWayPoint(wayPoint_pos, wayPoint_orient, false))
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

    _robotHand->setWayPoint(wayPoint_pos, wayPoint_orient);


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

            _robotHand->setStartingPose(starting_pos, starting_orient);
            _robotHand->setEndPose(end_pos, end_orient);
            _nGrid++;
            _robotHand->setWayPoint(starting_pos, starting_orient);
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

void TappingExplorationThread::moveArmUp()
{
    Vector wayPointPos, wayPointOrient;
    Vector armPos, orient;
    moveExplorationFinger(10, _curAbduction);

   //_robotHand->relaxTolerence();
    _robotHand->getPose(armPos, orient);
    //_robotHand->getStartingPose(startingPos, startingOrient);

    // TODO: rename vairables as needed.
    _robotHand->getWayPoint(wayPointPos, wayPointOrient, false);

    //Vector desiredArmPos;
    //_explorationFinger->toArmPosition(wayPointPos, desiredArmPos);
    armPos[2] = wayPointPos[2]; // Move the fingertip up to avoid collisiont
    //_objectFeatures->moveArmToPosition(armPos, orient);
    _robotHand->goToPoseSync(armPos, orient, 10);

    cout << "Waiting for force to return to normal value...";
    cout.flush();
    double force = _explorationFinger->getContactForce();

    if(force > 0.25)
    {
        yarp::os::Bottle msg, response;
        msg.clear();
        msg.addString("calib");
        _skinManagerCommand.write(msg, response);
        cout << response.toString() << endl;

    }
    while(force > 0.25)
    {
        force = _explorationFinger->getContactForce();
        for( int i = 0; i < 9; i++)
            force += _explorationFinger->getContactForce();
        force = force/10;
    }

    cout << "...done!" << endl;

    //_robotHand->strictTolerence();
    // move to the next location
}

bool TappingExplorationThread::threadInit()
{



    //return true;
}


void TappingExplorationThread::threadRelease()
{

    _skinManagerCommand.close();
}


} // namespace objectExploration
