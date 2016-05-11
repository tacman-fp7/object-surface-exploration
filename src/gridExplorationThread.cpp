#include "gridExplorationThread.h"
#include <iostream>

namespace objectExploration
{

using std::cerr;
using std::endl;
using std::cout;

void GridExplorationThread::run()
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
            TappingExplorationThread::approachObject();
            break;
        case CALCULATE_NEWWAYPONT:
#if DEBUG_LEVEL>=1
            cout << "Contact state is: new waypoint" << endl;
#endif
            TappingExplorationThread::calculateNewWaypoint();
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



    //yarp::os::Time::delay(1);  //TODO: Test if this is needed


}

void GridExplorationThread::moveToNewLocation()
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

void GridExplorationThread::maintainContact()
{
    // Log the data

    Vector fingertipPosition;
    _explorationFinger->getPosition(fingertipPosition);


    TappingExplorationThread::maintainContact();
}

bool GridExplorationThread::threadInit()
{

    if(_contactSafetyThread == NULL){
        try{
            _contactSafetyThread = new ContactSafetyThread(5, _robotHand );
        }
        catch(std::exception& e){
            std::cerr << e.what();
            std::cerr << "Disable contact safetey if you do not want to use it" << endl;
            return false;
        }

        if(!_contactSafetyThread->start())
        {
            return false;
        }
        else{
            cout << "Contact safety started" << endl;
        }
    }


    return true;
}


void GridExplorationThread::threadRelease()
{
    if(_contactSafetyThread != NULL)
    {
        if(_contactSafetyThread->isRunning())
            _contactSafetyThread->stop();
        delete(_contactSafetyThread);
        _contactSafetyThread = NULL;
    }

}


}// end of namespace
