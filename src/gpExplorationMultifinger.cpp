#include "gpExplorationMultifinger.h"
#include <yarp/os/Time.h>

namespace objectExploration {


using std::cout;
using std::endl;

void GPExplorationMultifingerThread::run()
{

    Vector startingPos, endingPos, startingOrient, endingOrient;
    _robotHand->getStartingPose(startingPos, startingOrient);
    _robotHand->getEndPose(endingPos, endingOrient);

    GPExplorationThread::initialiseGP(startingPos, startingOrient, endingPos, endingOrient);


    // Get a new waypoint from the GP model
    _contactState = SET_WAYPOINT_GP;


    _repeats = 0;
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
            //if(_contactState == MOVE_LOCATION){
            //    _stateMutex.unlock();
            //}
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

            if(_refineModel){
                GPExplorationThread::maintainContact_GP_Refine();
                _stateMutex.unlock();
            }
            else if(_validatePositionsEnabled){
                GPExplorationThread::maintainContact_GP_validatePosition();
                _stateMutex.unlock();
            }
            else{
                 GPExplorationThread::maintainContact();
                 multifingerContact();
            }
            //_stateMutex.unlock();
            break;
        case MOVE_LOCATION:
            cout << "Contact state is: move location" << endl;
            // Update the GP
            // Set the waypoint to the next waypoint suggested by the GP
            _stateMutex.unlock();
            GPExplorationThread::moveToNewLocation();
            break;
        case SET_WAYPOINT_GP:
            // Use the GP Model to set the next waypoint
            // TODO: determine whether we sould terminate or not
            _stateMutex.lock();
            cout << "ContactState is: get waypoint from GP" << endl;
            if(_refineModel){
                cout << "Refine" << endl;
                GPExplorationThread::setWayPoint_GP_Refine();
            }
            else if(_validatePositionsEnabled){
                cout << "Validate" << endl;
                GPExplorationThread::setWayPoint_GP_validate();
            }
            else{
                cout << "Normal" << endl;
                GPExplorationThread::setWayPoint_GP();
            }
            break;
            //case SET_WAYPOINT_REFINE_CONTACT:
            //    setWayPoint_GP_Refine();
            //    break;
        case REFINE_CONTACT:
            GPExplorationThread::maintainContact_GP_Refine();
            break;
        case VALIDATE_CONTACT:
            GPExplorationThread::maintainContact_GP_validatePosition();
            break;
        case FINISHED:
            cout << "Contact state is: finished" << endl;
            // I have to implement exit the thread procedure here
            //TappingExplorationThread::finshExploration();
            break;
        }
    }

    yarp::os::Time::delay(1);


}

bool GPExplorationMultifingerThread::clenchFinger(Finger *finger, double maxAngle)
{
    bool ret = false;
    cout << "Checking the contact...";


    /////// Move finger to starting position
    /// initial angle
    /// end angle
    /// force threshold




    // Move the finger
    moveFingerBlocking(finger, 10/2, _curAbduction, 40);

    Vector fingerAngles;
    double angle;
    for (int i = 10; i < maxAngle * 2; i++)
    {
        angle = i/2;
        moveFinger(finger, angle, _curAbduction);

        while(!finger->checkMotionDone()){

            finger->getAngels(fingerAngles);

            if(finger->getContactForce() >= _forceThreshold){
                cout << "contact confirmed" << endl;
                ret = true;
                break;
            }
            else if(fingerAngles[1] < 1){
                cout << "...[exceeded angle]..." << endl;
                ret = true;
                break;
            }
        }
        if(ret == true){
            break;
        }
    }


    if(ret == false){
        moveFingerBlocking(finger, 10/2, _curAbduction, 40);
    }
    return ret;

}


void GPExplorationMultifingerThread::multifingerContact(){
    // Step 1: move the finger
    // Step 2: detect contact
    // Step three register the location
    // Can I move all of them in parallel

    Finger *finger = _robotHand->getMiddleFinger();

    if(clenchFinger(finger, 80)){
        Vector fingerPos;
        finger->getPosition(fingerPos);
        std::cout << "Finger pos: " << fingerPos.toString() << std::endl;
    }

    // Move back to the starting position
    finger->setAngles(0, 0, 60);


}

bool GPExplorationMultifingerThread::threadInit()
{
    bool ret;

    ret = GPExplorationThread::threadInit();
    GPExplorationThread::disableSurfaceSampling();

    return ret;

}

void GPExplorationMultifingerThread::threadRelease()
{
    GPExplorationThread::threadRelease();

}



} // end of namespace
