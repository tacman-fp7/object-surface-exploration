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
    _contactState = SET_WAYPOINT_GP;
}

void GPExplorationThread::maintainContact()
{

    cout << "Contact state is GP maintain contact" << endl;
    // Store the contact location
    Vector fingertipPosition, fingertipOrientation;
    _objectFeatures->getFingertipPose(fingertipPosition, fingertipOrientation);
    _surfaceModel->addContactPoint(fingertipPosition);

    // Maintain contact
    TappingExplorationThread::maintainContact();



}



void GPExplorationThread::setWayPoint_GP()
{
    // Use the GP model to calculate a new waypoint

    Vector maxVariancePos;
    Vector orient;


    // Get the valid point from object features
    _objectFeatures->getWayPoint(maxVariancePos, orient, false);

    // I have to make sure the new waypoint is valid
    if(_surfaceModel->getMaxVariancePose(maxVariancePos))
        _objectFeatures->setWayPoint(maxVariancePos, orient);

    _contactState = APPROACH_OBJECT;
}

bool GPExplorationThread::threadInit()
{
    return TappingExplorationThread::threadInit();
}

void GPExplorationThread::threadRelease()
{

}


} // end of namespace
