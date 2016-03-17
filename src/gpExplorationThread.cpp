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

    // Check if finger position is close to the waypoint
    Vector pos, orient;
    Vector posInArm;

    _objectFeatures->getWayPoint(pos, orient, false);

    _objectFeatures->indexFinger2ArmPosition(fingertipPosition, posInArm);

    double dist;

    dist = sqrt(pow(pos[0] - posInArm[0], 2) +
            pow(pos[1] - posInArm[1], 2));
    if(dist > 8.0/1000)
    {
        cout << "Too far from the requested position: " << dist << endl;
    }



    _surfaceModel->addContactPoint(fingertipPosition);
    //_surfaceModel->addContactPoint(fingertipPosition);



    // Maintain contact

    TappingExplorationThread::maintainContact();


    // Wiggle wiggle

    _objectFeatures->getIndexFingertipPosition(fingertipPosition);
    if((fabs(fingertipPosition[2]  + 0.15) > 10.0/1000))
    {
        cout << "wiggle wiggle" << endl;


        // Wiggle
        // _contactSafetyThread->start();
        sampleSurface_wiggleFingers();
        // _contactSafetyThread->stop();

    }
    //_contactState = MOVE_LOCATION;


}

bool GPExplorationThread::confrimContact(double maxAngle)
{
    cout << "Checking the contact(wiggle wiggle)...";
    cout.flush();

    // Move the finger s
    _objectFeatures->fingerMovePosition(11, 0);
    _objectFeatures->fingerMovePosition(12, 15);
    while (!_objectFeatures->checkOpenHandDone())
        ;


    Vector indexFingerAngles;
    double angle;
    bool contact = false;
    for (int i = 0; i < maxAngle * 5; i++)
    {
        angle = i/5;
        _objectFeatures->fingerMovePosition(11, angle);


        while(!_objectFeatures->checkOpenHandDone())
        {
            _objectFeatures->getIndexFingerAngles(indexFingerAngles);

            if(_objectFeatures->getContactForce() >= _forceThreshold)
            {
                cout << "contact confirmed" << endl;
                contact = true;
                break;
            }
            else if(indexFingerAngles[1] < 3)
            {
                cout << "...[exceeded angle]..." << endl;
                contact = true;
                break;
            }
        }
        if(contact)
            break;

    }



    if (indexFingerAngles[0] > 3)
        contact = false;

    return contact;

}

void GPExplorationThread::sampleSurface_wiggleFingers()
{

    // Get the fingertip position
    Vector indexFingerAngles;
    Vector fingertipPosition;
    _objectFeatures->getIndexFingertipPosition(fingertipPosition);

    // set the current fingertip position as the next waypoint;
    Vector desiredArmPos, desiredArmOrient;
    _objectFeatures->getStartingPose(desiredArmPos, desiredArmOrient);

    // Open the fingertip

    _objectFeatures->fingerMovePosition(11, 0);
    _objectFeatures->fingerMovePosition(12, 15);
    while (!_objectFeatures->checkOpenHandDone())
        ;

    // Get the current desired arm positionw with the new fingertip configuration
    _objectFeatures->indexFinger2ArmPosition(fingertipPosition, desiredArmPos);


    // Sotore current arm position
    Vector prevArmPos, prevArmOrient;
    _objectFeatures->getArmPose(prevArmPos, prevArmOrient);

    desiredArmPos[2] -= 4.0/1000; // offset from the middle
    moveArmToWayPoint(desiredArmPos, desiredArmOrient);
    bool contact = false;

    while(!contact && !_contactSafetyThread->collisionDetected())
    {
        // cout << "move to way point" << endl;



        //cout << "confirming contact" << endl;

        desiredArmPos[2] -= 1.0/1000; // offset from the middle
        _objectFeatures->fingerMovePosition(11, 0, 50);
        _objectFeatures->fingerMovePosition(12, 15, 50);
        while (!_objectFeatures->checkOpenHandDone())
            ;
        moveArmToWayPoint(desiredArmPos, desiredArmOrient);
        contact = confrimContact(30);

    }

    if(contact)
    {
        cout << "We can sample the surface" << endl;
        _objectFeatures->fingerMovePosition(11, 10, 10);

        _objectFeatures->fingerMovePosition(7, 60, 30);
        yarp::os::Time::delay(2);
        _objectFeatures->fingerMovePosition(7, 0, 30);
        yarp::os::Time::delay(2);
    }

    // Move up

    //desiredArmPos[2] += 15.0/1000; // offset from the middle
    _objectFeatures->moveArmToPosition(prevArmPos, desiredArmOrient);


    _robotCartesianController->waitMotionDone(0.1, 20);


    // Now  make contact

    _contactState = STOP;

    /*
     * /////

    double offset = 3.0/1000;


    while(true){
        desiredArmPos[2] -= offset; // offset from the middle
        _robotCartesianController->goToPoseSync(desiredArmPos, desiredArmOrient);
       // _robotCartesianController->waitMotionDone(0.1, 2);





        bool motionDone = false;
        while(!motionDone)
        {
            if(_objectFeatures->getContactForce() > 3)
            {
                cout  << "Abandoned motion due to force" << endl;
                _robotCartesianController->stopControl();

                break;
            }

            _robotCartesianController->checkMotionDone(&motionDone);
        }

        _objectFeatures->getIndexFingertipPosition(fingertipPosition);
        cout << "AF: " << fingertipPosition.toString() << endl;



        _objectFeatures->getIndexFingerAngles(indexFingerAngles);

        if(indexFingerAngles[1] > 3 )
        {
            cout << "No contact!" << endl;
            cout << "Angles: " << indexFingerAngles.toString() << endl;

            offset += 3.0/1000;

            continue;
        }
        else
        {
            offset = 3.0/1000;
            break;
        }


    }

   _objectFeatures->fingerMovePosition(7, 60);
   yarp::os::Time::delay(2);
   _objectFeatures->fingerMovePosition(7, 0);
   yarp::os::Time::delay(2);

    // Just to be safe
   //////////////////// move the arm up

    Vector startingPos, startingOrient;
    Vector armPos, orient;



    _objectFeatures->getArmPose(armPos, orient);
    _objectFeatures->getStartingPose(startingPos, startingOrient);


    _objectFeatures->indexFinger2ArmPosition(startingPos, desiredArmPos);
    armPos[2] = desiredArmPos[2] + 10.0/1000; // Move the fingertip up to avoid collisiont
    _objectFeatures->moveArmToPosition(armPos, orient);


    _objectFeatures->setProximalAngle(10); // Needed by apporach object method */

}

void GPExplorationThread::moveArmToWayPoint(yarp::sig::Vector pos, yarp::sig::Vector orient)
{
    Vector indexFingerAngles;
    if(_robotCartesianController->goToPoseSync(pos, orient))
    {
        bool motionDone = false;
        while(!motionDone)
        {
            if(_objectFeatures->getContactForce() > _forceThreshold)
            {
                cout  << "Abandoned motion due to force" << endl;
                _robotCartesianController->stopControl();

                break;
            }

            _objectFeatures->getIndexFingerAngles(indexFingerAngles);

            if(indexFingerAngles[1] < 5 )
            {
                //cout << "Abandoned motion due to angles" << endl;
                //_robotCartesianController->stopControl();
                cout << "Angles: " << indexFingerAngles.toString() << endl;

               // break;
            }
            _robotCartesianController->checkMotionDone(&motionDone);
        }
    }
}

bool GPExplorationThread::initialiseGP(Vector startingPos, Vector startingOrient,
                                       Vector endingPos, Vector endingOrient)
{
    //bool ret = true;

    //_surfaceModel->loadContactData("boundingBox");
    double xMin, xMax, yMin, yMax, zMin;
    int nSteps = 20;
    xMax = startingPos[0];
    xMin = xMax - 70.0/1000;

    zMin = -0.15; // TODO: fix it! Maybe take it form reachable space
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
    _surfaceModel->setBoundingBox(nSteps, 0/1000);
    _surfaceModel->trainModel();
    _surfaceModel->updateSurfaceEstimate();
    //_surfaceModel->padBoundingBox(xMin, xMax, yMin, yMax, zMin, nSteps, 0.0/1000);

    // Set the waypoint to the midpoint
    Vector pos;
    pos.resize(3);
    pos[0] = (xMin + xMax)/2.0;
    pos[1] = (yMin + yMax)/2.0;
    pos[2] = startingPos[2];
    /* makeSingleContact(pos, startingOrient);

    makeSingleContact(pos, startingOrient);
    _surfaceModel->trainModel();
    _surfaceModel->updateSurfaceEstimate();

*/

    return true;



    /*   //////
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



    return ret; */
}


void GPExplorationThread::moveArmUp()
{
    Vector startingPos, startingOrient;
    Vector armPos, orient;
    TappingExplorationThread::moveIndexFinger(10);


    _objectFeatures->getArmPose(armPos, orient);
    _objectFeatures->getStartingPose(startingPos, startingOrient);

    Vector desiredArmPos;
    _objectFeatures->indexFinger2ArmPosition(startingPos, desiredArmPos);
    armPos[2] = desiredArmPos[2]; // Move the fingertip up to avoid collisiont
    _objectFeatures->moveArmToPosition(armPos, orient);


    cout << "Waiting for force to return to normal value...";
    cout.flush();
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
    Vector desiredArmPos;
    _objectFeatures->indexFinger2ArmPosition(pos, desiredArmPos);

    _objectFeatures->setWayPointGP(desiredArmPos, orient);
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

    Vector armPos, armOrient;
    Vector startingPos, startingOrient;


    //_objectFeatures->prepHand();

    _objectFeatures->getArmPose(armPos, armOrient);
    _objectFeatures->getStartingPose(startingPos, startingOrient);


    _objectFeatures->indexFinger2ArmPosition(startingPos, desiredArmPos);
    armPos[2] = desiredArmPos[2]; // Move the fingertip up to avoid collisiont
    _objectFeatures->moveArmToPosition(armPos, orient);

    _curProximal = 10;
    _curDistal = 90 - _curProximal;
    logFingertipControl();

    _objectFeatures->setProximalAngle(_curProximal);



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
    if(_contactSafetyThread == NULL)
        _contactSafetyThread = new ContactSafetyThread(5, _objectFeatures, _robotCartesianController );
    _contactSafetyThread->start();
    return TappingExplorationThread::threadInit();
}

void GPExplorationThread::threadRelease()
{
    if(_contactSafetyThread != NULL)
    {
        if(_contactSafetyThread->isRunning())
            _contactSafetyThread->stop();
        delete(_contactSafetyThread);
        _contactSafetyThread = NULL;
    }


}


} // end of namespace
