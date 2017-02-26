#include "gpExplorationThread.h"
#include <iostream>
#include <yarp/os/Time.h>
#include <fstream>

#define DEBUG_LEVEL 1
//#define FORCE_TH 1.6

namespace objectExploration
{

using yarp::sig::Vector;
using std::cout;
using std::endl;


void GPExplorationThread::run()
{

    Vector startingPos, endingPos, startingOrient, endingOrient;
    _robotHand->getStartingPose(startingPos, startingOrient);
    _robotHand->getEndPose(endingPos, endingOrient);

    initialiseGP(startingPos, startingOrient, endingPos, endingOrient);


    // Get a new waypoint from the GP model
    _contactState = SET_WAYPOINT_GP;


    _repeats = 0;
    _curProximal = 0;
    //_contactForceThreshold = _explorationFinger->getContactForceThreshold();//FORCE_TH; //TODO: should be in a config file

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
                maintainContact_GP_Refine();
                _stateMutex.unlock();
            }
            else if(_validatePositionsEnabled){
                maintainContact_GP_validatePosition();
                _stateMutex.unlock();
            }
            else{
                maintainContact();
            }
            //_stateMutex.unlock();
            break;
        case MOVE_LOCATION:
            cout << "Contact state is: move location" << endl;
            // Update the GP
            // Set the waypoint to the next waypoint suggested by the GP
            _stateMutex.unlock();
            moveToNewLocation();
            break;
        case SET_WAYPOINT_GP:
            // Use the GP Model to set the next waypoint
            // TODO: determine whether we sould terminate or not
            _stateMutex.lock();
            cout << "ContactState is: get waypoint from GP" << endl;
            if(_refineModel){
                cout << "Refine" << endl;
                setWayPoint_GP_Refine();
            }
            else if(_validatePositionsEnabled){
                cout << "Validate" << endl;
                setWayPoint_GP_validate();
            }
            else{
                cout << "Normal" << endl;
                setWayPoint_GP();
            }
            break;
            //case SET_WAYPOINT_REFINE_CONTACT:
            //    setWayPoint_GP_Refine();
            //    break;
        case REFINE_CONTACT:
            maintainContact_GP_Refine();
            break;
        case VALIDATE_CONTACT:
            maintainContact_GP_validatePosition();
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

void GPExplorationThread::setWayPoint_GP_validate()
{
    Vector validationPosition;
    Vector orient;
    Vector armPos;
    bool ret = false;

    _curProximal = 10;
    _curAbduction = 60;

    moveExplorationFingerBlocking(_curProximal, _curAbduction, 40);
    // Get the valid point from object features
    _robotHand->getWayPoint(armPos, orient, false);

    if( _surfaceModel->getNextValidationPosition(validationPosition))
    {

        _explorationFinger->toArmPosition(validationPosition, armPos);
        ret = _robotHand->setWayPointGP(armPos, orient);
    }
    else{
        _validatePositionsEnabled = false;
        _stateMutex.unlock();
    }

    TappingExplorationThread::moveArmUp();


    //yarp::os::Time::delay(5);
    if(ret){
        _contactState = APPROACH_OBJECT;
    }
    else{
        _contactState = MOVE_LOCATION;
    }


}

void GPExplorationThread::setWayPoint_GP_Refine()
{
    Vector refinementPosition;
    //if(!_surfaceModel->getNextRefinementPosition(refinementPosition))
    //    return;
    //cout << "Got a new position: " << refinementPosition.toString() << endl;



    Vector orient;
    Vector armPos;
    bool ret;

    _curProximal = 10;
    _curAbduction = 0;

    moveExplorationFingerBlocking(_curProximal, _curAbduction, 40);
    // Get the valid point from object features
    _robotHand->getWayPoint(armPos, orient, false);

    // I have to make sure the new waypoint is valid
    if( _surfaceModel->getNextRefinementPosition(refinementPosition))
    {
        //Get the current fingertip position
        //Vector tipPos, tipOrient;
        _explorationFinger->toArmPosition(refinementPosition, armPos);

        /*maxVariancePos[0] += pos[0];
        maxVariancePos[1] -= pos[1];
        maxVariancePos[2] -= pos[2];*/
        ret = _robotHand->setWayPointGP(armPos, orient);
    }
    else
    {
        _refineModel = false;

        _stateMutex.unlock();
    }


    // Get the position of the hand
    // This cannot be done in parallel with indexFinger2ArmPosition calcuation

    TappingExplorationThread::moveArmUp();


    //yarp::os::Time::delay(5);
    if(ret){
        _contactState = APPROACH_OBJECT;
    }
    else{
        _contactState = SET_WAYPOINT_GP;
    }




}

void GPExplorationThread::maintainContact_GP_validatePosition(){

    // Read the position
    cout << "Contact state is GP maintain contact" << endl;
    // Store the contact location

    Vector fingertipPosition;
    _explorationFinger->getPositionCorrected(fingertipPosition);

    if(_surfaceModel->validatePosition(fingertipPosition)){
        _surfaceModel->trainModel();
        _surfaceModel->updateSurfaceEstimate();
    }

    _contactState = SET_WAYPOINT_GP;
}

void GPExplorationThread::maintainContact_GP_Refine()
{

    // Store the contact location
    /*Vector fingertipPosition, fingertipOrientation;
    _objectFeatures->getIndexFingertipPosition(fingertipPosition);

    _surfaceModel->addContactPoint(fingertipPosition);
    _surfaceModel->trainModel();
    _surfaceModel->updateSurfaceEstimate();

    */

    // For refine contact I do no allow multiple sampling
    int prevNRepeats = _nRepeats;
    _nRepeats = 0;
    _repeats = 0;
    this->maintainContact();

    _surfaceModel->trainModel();
    _surfaceModel->updateSurfaceEstimate();

    // Restore to the prvious value;
    _nRepeats = prevNRepeats;
    _repeats = 0;

    _contactState = SET_WAYPOINT_GP;

}

void GPExplorationThread::enableSurfaceSampling()
{
    //_contactState = SET_WAYPOINT_REFINE_CONTACT;

    _sampleSurface = true;
}

void GPExplorationThread::disableSurfaceSampling()
{
    _sampleSurface = false;
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


    Vector fingertipPosition;
    _explorationFinger->getPositionCorrected(fingertipPosition);
    _explorationFinger->logTactileCoP();

    cout << "Fingertip pos before adding:" << fingertipPosition.toString() << endl;
    _zPoints.push_back(fingertipPosition[2]);

    while (_zPoints.size() < 1) {

        moveExplorationFingerBlocking(0, _curAbduction, 40);

        if(TappingExplorationThread::confrimContact(30))
        {
            _explorationFinger->getPositionCorrected(fingertipPosition);
            _zPoints.push_back(fingertipPosition[2]);
        }
        else
        {
            _contactState = CALCULATE_NEWWAYPONT;
            _zPoints.clear();
            return;
        }

    }

    double median = getMedian(_zPoints);
    fingertipPosition[2] = median;
    _surfaceModel->addContactPoint(fingertipPosition, _explorationFinger->getFingerID());
    //logging the data
    _explorationFingerLog << fingertipPosition[0] << ", " << fingertipPosition[1] << ", " << fingertipPosition[2] << endl;

    _zPoints.clear();

    // Save tactile data for Max
    // Read the tactile data
    Bottle *msg;
    msg = _tactileData_in.read(true);
    if(!msg->isNull()){

        //std::ofstream myFile;
        //myFile.open("taxel.csv", std::ios::app);




        for (int i = 0; i < 11; i++){
            _explorationFingerTaxelResponseLog << msg->get(i).asDouble() << ", ";
        }
        _explorationFingerTaxelResponseLog << msg->get(12).asDouble() << endl;

    }
    else{
        cout << "No taxel data was available." << endl;
    }

    // Maintain contact
    TappingExplorationThread::maintainContact();

    // Wiggle wiggle
    if(false){//_sampleSurface){
        _explorationFinger->getPositionCorrected(fingertipPosition);
        cout << "Finger position: " << fingertipPosition[2] + 0.15 << endl;
        if((fabs(fingertipPosition[2]  + 0.15) > 15.0/1000)){
            cout << "wiggle wiggle" << endl;
            sampleSurface_wiggleFingers();
        }
    }
}

bool GPExplorationThread::confirmWiggleContact(double maxAngle)
{


    cout << "We are in cofirm contact" << endl;
    // Move the finger s

    //Open the finger
    _explorationFinger->setProximalAngle(0, 50);
    _explorationFinger->setDistalAngle(0, 50);

    while (!_explorationFinger->checkMotionDone())
        ;


    Vector indexFingerAngles;
    double angle;
    bool contact = false;
    for (int i = 0; i < maxAngle ; i++)
    {
        angle = i;
        //_objectFeatures->fingerMovePosition(INDEX_PROXIMAL, angle, 10);
        _explorationFinger->setProximalAngle(angle, 10);

        while(!_explorationFinger->checkMotionDone())
        {

            //cout << "Force: " << _explorationFinger->getContactForce() << endl;
            if(_explorationFinger->getContactForce() >= _explorationFinger->getContactForceThreshold() / 2.0 )
            {
                cout << "contact confirmed" << endl;
                contact = true;
                break;
            }

            /*if((indexFingerAngles[1] + indexFingerAngles[2]) < 6)
            {
                cout << "...[exceeded angle]..." << endl;
                contact = true;
                break;
            }*/

        }
        if(contact)
            break;

    }


    cout << "Force: " << _explorationFinger->getContactForce() << endl;
    if(_explorationFinger->getContactForce() >= _explorationFinger->getContactForceThreshold() / 2)
    {
        cout << "contact confirmed" << endl;
        contact = true;

    }

    //if(_objectFeatures->getIndexFingerAngles(indexFingerAngles))
    if(_explorationFinger->getAngels(indexFingerAngles))
    {
        if (indexFingerAngles[0] > 2)
            contact = false;
    }
    else
    {
        cout << "Failed to read finger angles" << endl;
        contact = true; // I don't know what to do yet
    }

    return contact;

}

void GPExplorationThread::sampleSurface_wiggleFingers()
{


    // Get the fingertip position
    Vector indexFingerAngles;
    Vector fingertipPosition;
    _explorationFinger->getPositionCorrected(fingertipPosition);


    Vector desiredArmPos, desiredArmOrient;
    _robotHand->getStartingPose(desiredArmPos, desiredArmOrient);

    // Open the fingertip

    _curProximal = 0;
    _curAbduction = 60;
    double curDistal = 0;
    moveExplorationFingerBlocking(_curProximal, curDistal, _curAbduction, 40);
    //_objectFeatures->fingerMovePosition(11, 0, 40);
    //_objectFeatures->fingerMovePosition(12, 0, 40);
    //while (!_objectFeatures->checkOpenHandDone())
    //    ;

    // Get the current desired arm positionw with the new fingertip configuration
    _explorationFinger->toArmPosition(fingertipPosition, desiredArmPos);

    desiredArmPos[0] += 7.0/1000; // This is a hack to adjust for the position being in the middle of the finger
    _curProximal = 0;
    curDistal = 15;
    _curAbduction = 0;
    moveExplorationFingerBlocking(_curProximal, curDistal, _curAbduction, 40);
    //_objectFeatures->fingerMovePosition(11, 0, 40);
    //_objectFeatures->fingerMovePosition(12, 15, 40);
    //while (!_objectFeatures->checkOpenHandDone())
    //    ;

    // reset the force torque sensor's basline
    cout << "Contact safety baseline rest" << endl;
    _contactSafetyThread->resetBaseline();

    desiredArmPos[2] -= 25.0/1000; // offset from the middle
    moveArmToWayPoint(desiredArmPos, desiredArmOrient);

    // Read the current arm position, it may have stopped before reaching
    // the desired position
    Vector dummy;
    _robotHand->getPose(desiredArmPos, dummy);
    desiredArmPos[2] -= 2.0/1000; // move just a little  more to adjust for the 15 degree angle

    bool contact = false;

    cout << "Checking the contact(wiggle wiggle)...";
    cout.flush();
    while(!contact && !_contactSafetyThread->collisionDetected())
    {
        // cout << "move to way point" << endl;

        // Move the fingertip up
        _curProximal = 0;
        curDistal = 0;
        _curAbduction = 60;
        moveExplorationFingerBlocking(_curProximal, curDistal, _curAbduction, 50);

        // _objectFeatures->fingerMovePosition(11, 0, 50);
        // _objectFeatures->fingerMovePosition(12, 0, 50);
        // while (!_objectFeatures->checkOpenHandDone())
        //     ;

        //cout << "confirming contact" << endl;
        //_objectFeatures->moveArmToPosition(desiredArmPos, desiredArmOrient);
        _robotHand->goToPoseSync(desiredArmPos, desiredArmOrient, 10);

        bool done = false;
        while( !done && !isStopping())
        {
            _robotHand->checkMotionDone(&done);
        }

        //
        if(_contactSafetyThread->collisionDetected())
        {
            cout << "Collision detected before confirming contact" << endl;
        }
        contact = confirmWiggleContact(10);

        //if(_explorationFinger->getContactForce() >= _contactForceThreshold)
        //    contact = true;



        // Move the arm a little more in the next round if needed.
        desiredArmPos[2] -= 2.0/1000;

    }

    if(contact)
    {
        // I should have a maintain contact thread here
        cout << "We can sample the surface" << endl;



        // _objectFeatures->updateContactState(SAMPLE_SURFACE);
        // yarp::os::Time::delay(1);


        _curProximal = 3;
        curDistal = 0;
        _curAbduction = 60;

        moveExplorationFingerBlocking(_curProximal, curDistal, _curAbduction, 40);
        yarp::os::Time::delay(1);
        _curAbduction = 0;
        moveExplorationFingerBlocking(_curProximal, curDistal, _curAbduction, 40);
        _objectFeatures->updateContactState(_contactState);

        //_objectFeatures->updateContactState(_contactState);
        _curProximal = 60;
        moveExplorationFingerBlocking(_curProximal, curDistal, _curAbduction, 40);



        /*
        _objectFeatures->fingerMovePosition(11, 3, 10);
        _objectFeatures->fingerMovePosition(7, 60, 10);
        yarp::os::Time::delay(2);
        _objectFeatures->fingerMovePosition(7, 0, 10);
        yarp::os::Time::delay(2);
        */
    }
    else {
        cout << "We had no contact with the tip" << endl;

    }

    // Move up

    //desiredArmPos[2] += 15.0/1000; // offset from the middle
    // Get the starting pose
    _contactSafetyThread->suspend();
    //_objectFeatures->openHand();

    _curProximal = 0; curDistal = 0; _curAbduction = 60;
    moveExplorationFingerBlocking(_curProximal, curDistal, _curAbduction, 40);

    Vector startingPos, startingOrient;
    _robotHand->getStartingPose(startingPos, startingOrient);
    desiredArmPos[2] += 60.0/1000;
    _robotHand->goToPoseSync(desiredArmPos, startingOrient,0);

    bool done = false;

    while(!done && (_contactState != STOP))
    {
        _robotHand->checkMotionDone(&done);
    }

    _contactSafetyThread->resume();


    _curProximal = 0; curDistal = 0; _curAbduction = 0;
    moveExplorationFingerBlocking(_curProximal, curDistal, _curAbduction, 40);


}

void GPExplorationThread::moveArmToWayPoint(yarp::sig::Vector pos, yarp::sig::Vector orient)
{
    Vector indexFingerAngles;
    if(_robotHand->goToPoseSync(pos, orient))
    {
        bool motionDone = false;
        while(!motionDone)
        {
            if(_explorationFinger->getContactForce() > _explorationFinger->getContactForceThreshold())
            {
                cout  << "Abandoned motion due to force" << endl;
                _robotHand->stopControl();

                break;
            }

            _explorationFinger->getAngels(indexFingerAngles);
            //_objectFeatures->getIndexFingerAngles(indexFingerAngles);

            if(indexFingerAngles[1] < 2 )
            {
                //cout << "Abandoned motion due to angles" << endl;
                _robotHand->stopControl();
                cout << "Angles: " << indexFingerAngles.toString() << endl;

                // break;
            }
            _robotHand->checkMotionDone(&motionDone);
        }
    }
}

bool GPExplorationThread::initialiseGP(Vector startingPos, Vector startingOrient,
                                       Vector endingPos, Vector endingOrient)
{
    //bool ret = true;

    //_surfaceModel->loadContactData("boundingBox");
    double xMin, xMax, yMin, yMax, zMin;
    t_explorationAreaParams explorationParams = _robotHand->getExplorationAreaParams();

    int nSteps = explorationParams.nSteps;  //20;
    xMax = startingPos[0];
    xMin = xMax - explorationParams.xWidth/1000.0;//170.0/1000;

    zMin = explorationParams.tableHeight; //-0.135;//-0.148; // Table height TODO: fix it! Maybe take it form reachable space

    ////////////////////////////////////////////////////////////////////////////
    //// The start and the end y locations depends on the right/left hand //////
    //// Set the appropriately                                            //////
    ////////////////////////////////////////////////////////////////////////////
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




    _surfaceModel->padBoundingBox(xMin, xMax, yMin, yMax, zMin, nSteps, 0.0/1000);
    _surfaceModel->setBoundingBox(nSteps, 0.0/1000);
    _surfaceModel->trainModel();
    _surfaceModel->updateSurfaceEstimate();


    return true;

}

double GPExplorationThread::getMedian(std::vector<double> &vec)
{


    size_t vecSize = vec.size();


    sort(vec.begin(), vec.end());
    double median;

    if(vecSize % 2 == 0){
        median = (vec.at(vecSize / 2 - 1) + vec.at(vecSize / 2)) / 2.0;
    }
    else{
        median = vec.at(vecSize / 2);
    }

}


void GPExplorationThread::setWayPoint_GP()
{
    // Use the GP model to calculate a new waypoint

    Vector maxVariancePos;
    Vector orient;
    Vector armPos;
    bool ret;

    _curProximal = 10;
    _curAbduction = 60;

    moveExplorationFingerBlocking(_curProximal, _curAbduction, 40);
    // Get the valid point from object features
    _robotHand->getWayPoint(armPos, orient, false);

    // I have to make sure the new waypoint is valid
    if(_surfaceModel->getMaxVariancePose(maxVariancePos))
    {
        //Get the current fingertip position
        //Vector tipPos, tipOrient;
        _explorationFinger->toArmPosition(maxVariancePos, armPos);

        /*maxVariancePos[0] += pos[0];
        maxVariancePos[1] -= pos[1];
        maxVariancePos[2] -= pos[2];*/
        ret = _robotHand->setWayPointGP(armPos, orient);
    }


    // Get the position of the hand
    // This cannot be done in parallel with indexFinger2ArmPosition calcuation

    TappingExplorationThread::moveArmUp();

    if(ret)
        _contactState = APPROACH_OBJECT;
    else
        _contactState = FINISHED;
}

bool GPExplorationThread::threadInit()
{

    // TODO: This should be moved to the base class
    string fileName = _explorationFinger->getFingerName() + "FingertipLogGP.csv";
    _explorationFingerLog.open(fileName.c_str());
    _explorationFingerTaxelResponseLog.open("taxel.csv"); //TODO: proper name
    fileName = _auxiliaryFinger->getFingerName() + "FingertipLogGP.csv";
    _auxiliaryFingerLog.open(fileName.c_str());


    if(_contactSafetyThread == NULL){
        try{
            _contactSafetyThread = new ContactSafetyThread(5, _robotHand );
        }
        catch(std::exception& e){
            std::cerr << e.what();
            std::cerr << "Disable contact safetey if you do not want to use it" << endl;
            return true;
        }

        if(!_contactSafetyThread->start())
        {
            return false;
        }
        else{
            cout << "Contact safety started" << endl;
        }
    }
    //_contactSafetyThread->start();
/*    _skinManagerCommand.open("/object-exploration/skinManager/rpc:o");
    yarp::os::Network::connect("/object-exploration/skinManager/rpc:o", "/skinManager/rpc");

*/
    // This to make Max's stuff work
    _tactileData_in.open("/object-exploration/skin/" + _robotHand->getArmName() + "_hand:i");
    yarp::os::Network::connect("/" + _robotHand->getRobotName() + "/skin/" + _robotHand->getArmName() +  "_hand",
                               "/object-exploration/skin/" + _robotHand->getArmName() + "_hand:i");


    return true;
}

void GPExplorationThread::threadRelease()
{

     // TODO: This should be moved to the base class
    _explorationFingerLog.close();
    _auxiliaryFingerLog.close();
    _explorationFingerTaxelResponseLog.close();

    if(_contactSafetyThread != NULL)
    {
        if(_contactSafetyThread->isRunning())
            _contactSafetyThread->stop();
        delete(_contactSafetyThread);
        _contactSafetyThread = NULL;
    }


}


} // end of namespace
