#include "gpExplorationThread.h"
#include <iostream>
#include <yarp/os/Time.h>

#define DEBUG_LEVEL 1
#define FORCE_TH 0.8

namespace objectExploration
{

using yarp::sig::Vector;
using std::cout;
using std::endl;

void GPExplorationThread::run()
{

    Vector startingPos, endingPos, startingOrient, endingOrient;
    _objectFeatures->getStartingPose(startingPos, startingOrient);
    _objectFeatures->getEndingPose(endingPos, endingOrient);
    initialiseGP(startingPos, startingOrient, endingPos, endingOrient);


    // Get a new waypoint from the GP model
    _contactState = SET_WAYPOINT_GP;

    _repeats = 0;
    _curDistal = 0;
    _curProximal = 0;
    _forceThreshold = FORCE_TH; //TODO: should be in a config file

    while(!isStopping() && !(_contactState == STOP)) // Keep running this
    {
        _objectFeatures->updateContactState(_contactState);


        switch (_contactState)
        {

        case UNDEFINED:
            // This is the first round no approach has been made
            // Get the waypoint and set the state to approaching
            cout << "Contact state is: undefined" << endl;
            break;
        case  APPROACH_OBJECT:
            // Aproach and wait for contact
            // If there is contact, set the state to MAINTAIN_CONTACT
            // If there is no contact, set the state to CALCULATE_NEWWAYPOINT
            // If the limit is reached,  set the state to MOVELOCATION <= this needs to be changed for GP
            cout << "Contact state is: approach" << endl;
            TappingExplorationThread::approachObject();
            break;
        case CALCULATE_NEWWAYPONT:
            // Use the current position of the fingertip as the
            // next waypoint
            cout << "Contact state is: new waypoint" << endl;
            TappingExplorationThread::calculateNewWaypoint();
            break;
        case MAINTAIN_CONTACT:
            cout << "Contact state is: maintain contact" << endl;
            // Store update the contact location in the GP
            // Maintain contact for a couple of seconds
            // Set the state to GET_WAYPOINT_GP
            maintainContact();
            break;
        case MOVE_LOCATION:
            cout << "Contact state is: move location" << endl;
            // Update the GP
            // Set the waypoint to the next waypoint suggested by the GP
            moveToNewLocation();
            break;
        case SET_WAYPOINT_GP:
            // Use the GP Model to set the next waypoint
            // TODO: determine whether we sould terminate or not
            cout << "ContactState is: get waypoint from GP" << endl;
            setWayPoint_GP();
            break;
        case FINISHED:
            cout << "Contact state is: finished" << endl;
            // I have to implement exit the thread procedure here
            TappingExplorationThread::finshExploration();
            break;
        }
    }

    yarp::os::Time::delay(1);


}

void GPExplorationThread::moveToNewLocation()
{
    cout << "Contact state is: move location" << endl;
    _surfaceModel->trainModel();
    _surfaceModel->updateSurfaceEstimate();
    _contactState = SET_WAYPOINT_GP;
}

void GPExplorationThread::maintainContact()
{

    cout << "Contact state is GP maintain contact" << endl;
    // Store the contact location
    Vector fingertipPosition, fingertipOrientation;
    _objectFeatures->getIndexFingertipPosition(fingertipPosition);
    // _objectFeatures->getFingertipPose(fingertipPosition, fingertipOrientation);
    _surfaceModel->addContactPoint(fingertipPosition);

    // Maintain contact
    //TappingExplorationThread::maintainContact();
    _contactState = MOVE_LOCATION;


}

bool GPExplorationThread::initialiseGP(Vector startingPos, Vector startingOrient,
                                       Vector endingPos, Vector endingOrient)
{
    bool ret = true;

    Vector pos, orient;
    pos = startingPos;
    orient = startingOrient;

    // Move to the starting positio
    makeSingleContact(pos, orient);

    pos[0] -= 60.0/1000;
    makeSingleContact(pos, orient);

    pos = endingPos;
    orient = endingOrient;

    pos[0] -= 60.0/1000;
    makeSingleContact(pos, orient);

    pos = endingPos;
    makeSingleContact(pos, orient);

    pos = startingPos;
    pos[0] -= 20.0/1000;
    pos[1]  = (startingPos[1] + endingPos[1])/2;
    makeSingleContact(pos, orient);

    pos = startingPos;
    pos[0] -= 30.0/1000;
    pos[1]  = (startingPos[1] + endingPos[1])/2;
    makeSingleContact(pos, orient);

    pos = startingPos;
    pos[0] -= 40.0/1000;
    pos[1]  = (startingPos[1] + endingPos[1])/2;
    makeSingleContact(pos, orient);


    _surfaceModel->trainModel();
    _surfaceModel->setBoundingBox();
    _surfaceModel->updateSurfaceEstimate();

    return ret;
}

void GPExplorationThread::makeSingleContact(Vector pos, Vector orient)
{
    _objectFeatures->setWayPointGP(pos, orient);
    _contactState = APPROACH_OBJECT;

    while((_contactState != FINISHED) && !isStopping() && !(_contactState == STOP))
    {
        switch (_contactState)
        {
        case UNDEFINED:
            // This is the first round no approach has been made
            // Get the waypoint and set the state to approaching
            cout << "Contact state is: undefined" << endl;
            break;
        case  APPROACH_OBJECT:
            // Aproach and wait for contact
            // If there is contact, set the state to MAINTAIN_CONTACT
            // If there is no contact, set the state to CALCULATE_NEWWAYPOINT
            // If the limit is reached,  set the state to MOVELOCATION <= this needs to be changed for GP
            cout << "Contact state is: approach" << endl;
            TappingExplorationThread::approachObject();
            break;
        case CALCULATE_NEWWAYPONT:
            // Use the current position of the fingertip as the
            // next waypoint
            cout << "Contact state is: new waypoint" << endl;
            TappingExplorationThread::calculateNewWaypoint();
            break;
        case MAINTAIN_CONTACT:
            cout << "Contact state is: maintain contact" << endl;
            // Store update the contact location in the GP
            // Maintain contact for a couple of seconds
            // Set the state to GET_WAYPOINT_GP
            maintainContact();
            break;
        case MOVE_LOCATION:
            cout << "Contact state is: move location" << endl;
            // Update the GP
            // Set the waypoint to the next waypoint suggested by the GP
            // moveToNewLocation();
            _contactState = FINISHED;
            break;
        case SET_WAYPOINT_GP:
            // Use the GP Model to set the next waypoint
            // TODO: determine whether we sould terminate or not
            cout << "ContactState is: get waypoint from GP" << endl;
            //setWayPoint_GP();
            _contactState = FINISHED;
            break;
        case FINISHED:
            cout << "Contact state is: finished" << endl;
            // I have to implement exit the thread procedure here
            //TappingExplorationThread::finshExploration();
            break;
        }
    }

    Vector armPos, armOrient;
    Vector startingPos, startingOrient;
    _objectFeatures->prepHand();
    _objectFeatures->getArmPose(armPos, armOrient);
    _objectFeatures->getStartingPose(startingPos, startingOrient);

    armPos[2] = startingPos[2]; // Move the fingertip up to avoid collisiont
    _objectFeatures->moveArmToPosition(armPos, orient);



}

void GPExplorationThread::setWayPoint_GP()
{
    // Use the GP model to calculate a new waypoint

    Vector maxVariancePos;
    Vector orient;
    Vector armPos;
    bool ret;

    _objectFeatures->prepHand();
    while(!_objectFeatures->checkOpenHandDone() && !isStopping())
        ;
    // Get the valid point from object features
    _objectFeatures->getWayPoint(armPos, orient, false);

    // I have to make sure the new waypoint is valid
    if(_surfaceModel->getMaxVariancePose(maxVariancePos))
    {
        //Get the current fingertip position
        //Vector tipPos, tipOrient;
        _objectFeatures->indexFinger2ArmPosition(maxVariancePos, armPos);

        /*maxVariancePos[0] += pos[0];
        maxVariancePos[1] -= pos[1];
        maxVariancePos[2] -= pos[2];*/
        ret = _objectFeatures->setWayPointGP(armPos, orient);
    }


    // Get the position of the hand
    // This cannot be done in parallel with indexFinger2ArmPosition calcuation

    Vector startingPos, startingOrient;
    _objectFeatures->getArmPose(armPos, orient);
    _objectFeatures->getStartingPose(startingPos, startingOrient);

    armPos[2] = startingPos[2]; // Move the fingertip up to avoid collisiont
    _objectFeatures->moveArmToPosition(armPos, orient);
    //_objectFeatures->
    if(ret)
        _contactState = APPROACH_OBJECT;
    else
        _contactState = FINISHED;
}

bool GPExplorationThread::threadInit()
{
    return TappingExplorationThread::threadInit();
}

void GPExplorationThread::threadRelease()
{

}


} // end of namespace
