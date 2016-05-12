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


    _curProximal = 10;
    _curAbduction = 0;
    moveIndexFingerBlocking(_curProximal, _curAbduction, 40);

    // Get the next point
    Vector wayPoint;
    Vector startingPos, startingOrient;
    _robotHand->getStartingPose(startingPos, startingOrient);

    if(_objectModel->getNextSamplingPos(wayPoint)){
        wayPoint[2] = startingPos[2]; // Set the z position to starting point+
        _robotHand->setWayPoint(wayPoint);
        _contactState = APPROACH_OBJECT;
    }
    else{
        _contactState = FINISHED;
    }

    // Move the finger up
    TappingExplorationThread::moveArmUp();

}

void GridExplorationThread::maintainContact()
{
    // Log the data

    Vector fingertipPosition;
    _explorationFinger->getPosition(fingertipPosition);
    _objectModel->addContactPoint(fingertipPosition);
    _objectModel->saveContactPoints();

    TappingExplorationThread::maintainContact();
}

bool GridExplorationThread::threadInit()
{

    if(_contactSafetyThread == NULL){
        try{
            _contactSafetyThread = new ContactSafetyThread(5, _robotHand ); //TODO: config file
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

    if(_objectModel != NULL){
        Vector startingPos, startingOrient;
        Vector endingPos, endingOrient;
        _robotHand->getStartingPose(startingPos, startingOrient);
        _robotHand->getEndPose(endingPos, endingOrient);

        _objectModel->init(startingPos, endingPos);

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

    if(_objectModel != NULL){
        delete _objectModel;
        _objectModel = NULL;
    }
}


}// end of namespace
