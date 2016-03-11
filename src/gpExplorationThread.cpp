#include "gpExplorationThread.h"
#include <iostream>
#include <yarp/os/Time.h>

#define DEBUG_LEVEL 1
#define FORCE_TH 1.6

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
    //cout << "Position of the fingertip: " << fingertipPosition.toString();

    _surfaceModel->addContactPoint(fingertipPosition);

    // Maintain contact
    TappingExplorationThread::maintainContact();
    //_contactState = MOVE_LOCATION;


}

bool GPExplorationThread::initialiseGP(Vector startingPos, Vector startingOrient,
                                       Vector endingPos, Vector endingOrient)
{
    bool ret = true;

    _surfaceModel->loadContactData("boundingBox");
    _surfaceModel->trainModel();
    _surfaceModel->setBoundingBox(120, 5.0/1000);
    _surfaceModel->updateSurfaceEstimate();

    return true;



    //////
    Vector pos, orient;
    pos = startingPos;
    orient = startingOrient;
    double xSteps = -20.0/1000;
    double ySteps = 0;
    int nXGrid = 4;
    int nYGrid = 5;

    if(startingPos[1] < endingPos[1])
        ySteps = 20.0/1000;
    else
        ySteps = -20.0/1000;

    // Make sure the first point is valid
    _objectFeatures->setWayPoint(pos, orient);
    while(pos[0] >= (startingPos[0] + xSteps * nXGrid))
    {

        for (int i = 0; i < nYGrid; i++)
        {

            makeSingleContact(pos, orient);
            pos[1] += ySteps;
        }
        pos[0] += xSteps;
        pos[1] = startingPos[1];

    }


    double xMax = startingPos[0];
    double xMin = startingPos[0] + xSteps * nXGrid;
    double yMin, yMax;

    if(startingPos[1] < endingPos[1])
    {
        yMin = startingPos[1];
        yMax = endingPos[1];
    }
    else
    {
        yMin = endingPos[1];
        yMax = startingPos[1];
    }

    _surfaceModel->padBoundingBox();
    _surfaceModel->trainModel();
   // _surfaceModel->setBoundingBox(xMin, xMax, yMin, yMax, 120, 5.0/1000);
    _surfaceModel->setBoundingBox(120, 5.0/1000);
    _surfaceModel->updateSurfaceEstimate();



    return ret;
}


void GPExplorationThread::moveArmUp()
{
    Vector startingPos, startingOrient;
    Vector armPos, orient;
    TappingExplorationThread::moveIndexFinger(10);


    _objectFeatures->getArmPose(armPos, orient);
    _objectFeatures->getStartingPose(startingPos, startingOrient);

    armPos[2] = startingPos[2]; // Move the fingertip up to avoid collisiont
    _objectFeatures->moveArmToPosition(armPos, orient);


    cout << "Waiting for force to return to normal value...";
    double force = _objectFeatures->getContactForce();

    while(force > 0.5)
    {
        force = _objectFeatures->getContactForce();
        for( int i = 0; i < 9; i++)
            force += _objectFeatures->getContactForce();
        force = force/10;
    }

    cout << "done!" << endl;
}

void GPExplorationThread::makeSingleContact(Vector pos, Vector orient)
{
    //Vector armPos;
     //_objectFeatures->indexFinger2ArmPosition(pos, armPos);
    _objectFeatures->setWayPointGP(pos, orient);
    _contactState = APPROACH_OBJECT;

    moveArmUp();
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

    Vector  armPos, armOrient;
    Vector startingPos, startingOrient;
    //_objectFeatures->prepHand();
    _curProximal = 10;
    _curDistal = 90 - _curProximal;
    logFingertipControl();

    _objectFeatures->setProximalAngle(_curProximal);
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

    _curProximal = 10;
    _curDistal = 90 - _curProximal;
    _objectFeatures->setProximalAngle(_curProximal);

    //_objectFeatures->prepHand();
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

    moveArmUp();

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
