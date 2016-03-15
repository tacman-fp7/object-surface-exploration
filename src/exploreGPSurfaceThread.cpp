#include <exploreGPSurfaceThread.h>

#define FORCE_TH 1.6

namespace objectExploration
{

using yarp::sig::Vector;
using std::cout;
using std::endl;

void ExploreGPSurfaceThread::run()
{

    // Assuming that there is a valid model saved
    // in the working directory
    Vector startingPos, endingPos, startingOrient, endingOrient;
    _objectFeatures->getStartingPose(startingPos, startingOrient);
    _objectFeatures->getEndingPose(endingPos, endingOrient);
    initialiseGP(startingPos, startingOrient, endingPos, endingOrient);

    _repeats = 0;
    _curDistal = 0;
    _curProximal = 0;
    _forceThreshold = FORCE_TH; //TODO: should be in a config file

    // Get a new waypoint from the GP model
    _contactState = SET_WAYPOINT_GP;

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


    }

bool ExploreGPSurfaceThread::initialiseGP(yarp::sig::Vector startingPos, yarp::sig::Vector startingOrient, yarp::sig::Vector endingPos, yarp::sig::Vector endingOrient)
{
    //double xMin, xMax, yMin, yMax, zMin;
    int nSteps = 20;


    _surfaceModel->loadContactData("GP");
    _surfaceModel->setBoundingBox(nSteps, 0/1000);
    _surfaceModel->trainModel();
    _surfaceModel->updateSurfaceEstimate();

}


} // end of namespace
