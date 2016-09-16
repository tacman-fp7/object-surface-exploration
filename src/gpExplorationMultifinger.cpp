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

bool GPExplorationMultifingerThread::clenchRingLittleFinger(Finger *ringFinger, Finger *littleFinger, double maxAngle, clenchResults_t *clenchResults)
{

    return true;
    bool ret = false;
    bool ringFingerForce, littleFingerForce, ringFingerExAngle, littleFingerExAngle;
    ringFingerForce = littleFingerForce = ringFingerExAngle = littleFingerExAngle = false;

    cout << "Checking the contact...";


    /////// Move finger to starting position
    /// initial angle
    /// end angle
    /// force threshold




    // Move the finger
    moveFingerBlocking(ringFinger, 40, _curAbduction, 60);

    Vector ringFingerAngles;
    Vector littleFingerAngles;

    double angle;
    for (int i = 40; i < maxAngle; i+=2)
    {
        angle = i;
        moveFinger(ringFinger, angle, _curAbduction);

        do{

            ringFinger->getAngels(ringFingerAngles);
            littleFinger->getAngels(littleFingerAngles);
            //cout << endl << "Ring  : " << ringFingerAngles.toString() << endl;
            //cout << "Little: " << littleFingerAngles.toString() << endl;


            if(ringFinger->getContactForce() >= _forceThreshold/4.0){
                cout << "Ring finger contact confirmed" << endl;
                ringFingerForce = true;

            }
            if(littleFinger->getContactForce() >= _forceThreshold/4.0){
                cout << "Little finger contact confirmed" << endl;
                littleFingerForce = true;
            }
            if(ringFingerAngles[0] < 10 ){
                cout << "...ring finger [exceeded angle]..." << endl;
                ringFingerExAngle = true;
            }
            if(littleFingerAngles[0] < 10){
                cout << "...little finger [exceeded angle]..." << endl;
                littleFingerExAngle = true;
            }

            if(ringFingerExAngle || littleFingerExAngle){
                break;
            }
        }while(!ringFinger->checkMotionDone());

        if(ret = ringFingerForce || ringFingerExAngle ||
                littleFingerForce || littleFingerExAngle){
            break;
        }
    }


    if(ret == false){
        cout << "no contact was detected" << endl;
        moveFingerBlocking(ringFinger, 0, _curAbduction, 40);
    }
    clenchResults->littleFingerExAngle  = littleFingerExAngle;
    clenchResults->littleFingerForce = littleFingerForce;
    clenchResults->ringFingerExAngle = ringFingerExAngle;
    clenchResults->ringFingerForce = ringFingerForce;

    return ret;

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

        do{

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
        } while(!finger->checkMotionDone());
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

  return;
    Finger *middleFinger = _robotHand->getMiddleFinger();
    Finger *ringFinger = _robotHand->getRingerFinger();
    Finger *littleFinger = _robotHand->getLittleFinger();

    clenchResults_t clenchResults;
    bool middleFingerHasContact;


    _contactMiddleFinger.initThread(middleFinger, 80, _forceThreshold);
    _contactMiddleFinger.start();

   // _contactRingAndLittleFingers.initThread(ringFinger, littleFinger, 120, _forceThreshold);
   // _contactRingAndLittleFingers.start();



    while(_contactMiddleFinger.isRunning() || _contactMiddleFinger.isRunning())
        ;

    _contactMiddleFinger.getResults(&middleFingerHasContact);
    //_contactRingAndLittleFingers.getResults(&clenchResults);



    if(middleFingerHasContact){
        Vector fingertipPosition;
        middleFinger->getPosition(fingertipPosition);
        std::cout << "Middle finger pos: " << fingertipPosition.toString() << std::endl;
        _surfaceModel->addContactPoint(fingertipPosition);
    }



    if(clenchResults.littleFingerExAngle || clenchResults.littleFingerForce
            || clenchResults.ringFingerExAngle || clenchResults.ringFingerForce){




        if(clenchResults.littleFingerForce || clenchResults.littleFingerExAngle){
            cout << "Little finger contact" << endl;
            /*Vector fingertipPosition;
            littleFinger->getPosition(fingertipPosition);
            std::cout << "Little finger pos: " << fingertipPosition.toString() << std::endl;
            _surfaceModel->addContactPoint(fingertipPosition);*/
        }
        if(clenchResults.ringFingerForce||clenchResults.ringFingerExAngle){

            cout << "Ring finger contact" << endl;
            /*Vector fingertipPosition;
            ringFinger->getPosition(fingertipPosition);
            std::cout << "Ring finger pos: " << fingertipPosition.toString() << std::endl;
            _surfaceModel->addContactPoint(fingertipPosition);*/

        }
    }


    // Move back to the starting position
    middleFinger->setAngles(0, 0, 60);
    ringFinger->setAngles(0, 0, 60);

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

bool RingAndLittleFingersContactThread::threadInit(){
    _clenchResults.littleFingerExAngle = false;
    _clenchResults.littleFingerForce = false;
    _clenchResults.ringFingerExAngle = false;
    _clenchResults.ringFingerForce = false;
}

void RingAndLittleFingersContactThread::run(){
    cout << "Checking the ring contact...";
    _clenchResults.littleFingerExAngle = false;
    _clenchResults.littleFingerForce = false;
    _clenchResults.ringFingerExAngle = false;
    _clenchResults.ringFingerForce = false;

    bool ret;

    //bool ringFingerForce, littleFingerForce, ringFingerExAngle, littleFingerExAngle;
    //ringFingerForce = littleFingerForce = ringFingerExAngle = littleFingerExAngle = false;




    /////// Move finger to starting position
    /// initial angle
    /// end angle
    /// force threshold

    /////// Move finger to starting position
    _ringFinger->setAngles(40, 60);
    while(!_ringFinger->checkMotionDone() && !isStopping() )
        ;

    Vector ringFingerAngles;
    Vector littleFingerAngles;


    for (int angle = 40; angle < _maxAngle; angle+=2)
    {

        _ringFinger->setAngles(angle, 60);

        do{

            _ringFinger->getAngels(ringFingerAngles);
            _littleFinger->getAngels(littleFingerAngles);
            //cout << endl << "Ring  : " << ringFingerAngles.toString() << endl;
            //cout << "Little: " << littleFingerAngles.toString() << endl;


            if(_ringFinger->getContactForce() >= _forceThreshold/4.0){
                cout << "Ring finger contact confirmed" << endl;
                _clenchResults.ringFingerForce = true;

            }
            if(_littleFinger->getContactForce() >= _forceThreshold/4.0){
                cout << "Little finger contact confirmed" << endl;
                _clenchResults.littleFingerForce = true;
            }
            if(ringFingerAngles[0] < 10 ){
                cout << "...ring finger [exceeded angle]..." << endl;
                _clenchResults.ringFingerExAngle = true;
            }
            if(littleFingerAngles[0] < 10){
                cout << "...little finger [exceeded angle]..." << endl;
                _clenchResults.littleFingerExAngle = true;
            }

            if(_clenchResults.ringFingerExAngle || _clenchResults.littleFingerExAngle){
                break;
            }
        }while(!_ringFinger->checkMotionDone());

        if(ret = _clenchResults.ringFingerForce  || _clenchResults.ringFingerExAngle ||
                _clenchResults.littleFingerForce || _clenchResults.littleFingerExAngle){
            break;
        }
    }


    if(ret == false){
        cout << "no contact was detected" << endl;
        _ringFinger->setAngles(40, 60);
        while(!_ringFinger->checkMotionDone() && !isStopping() )
            ;
    }
}

void MiddleFingerContactThread::initThread(Finger *finger, double maxAngle, double forceThreshold){
    _middleFinger = finger;
    _maxAngle = maxAngle;
    _forceThreshold = forceThreshold;}

void MiddleFingerContactThread::run(){
    cout << "Checking the contact...";
    _contactState = false;
    /////// Move finger to starting position
    _middleFinger->setAngles(10/2, 60);
    while(!_middleFinger->checkMotionDone() && !isStopping() )
        ;


    Vector fingerAngles;
    double angle;
    for (int i = 10; i < _maxAngle * 2; i++)
    {
        angle = i/2;
        _middleFinger->setAngles(angle, 60);

        do{

            _middleFinger->getAngels(fingerAngles);

            if(_middleFinger->getContactForce() >= _forceThreshold){
                cout << "contact confirmed" << endl;
                _contactState = true;
                break;
            }
            else if(fingerAngles[1] < 1){
                cout << "...[exceeded angle]..." << endl;
                _contactState = true;
                break;
            }
        } while(!_middleFinger->checkMotionDone());

        if(_contactState == true){
            break;
        }
    }


    if(_contactState == false){
        _middleFinger->setAngles(10/2, 60);
        while(!_middleFinger->checkMotionDone() && !isStopping() )
            ;
    }
}

} // end of namespace
