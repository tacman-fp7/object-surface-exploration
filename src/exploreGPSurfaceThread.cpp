#include <exploreGPSurfaceThread.h>

#include <yarp/os/Time.h>

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
    //_curDistal = 0;
    _curProximal = 0;
    _forceThreshold = FORCE_TH; //TODO: should be in a config file

    // Get a new waypoint from the GP model
    _contactState = SET_WAYPOINT_GP;




    while(!isStopping() && !(_contactState == STOP)) // Keep running this
    {

        /* Vector maxPoint;
        if(_surfaceModel->getNextSamplingPosition(maxPoint))
        {
            cout << maxPoint.toString() << endl;
        }
        */

        // yarp::os::Time::delay(1);

        _objectFeatures->updateContactState(_contactState);

    Vector fingertipPositon;


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


            //_objectFeatures->getIndexFingertipPosition(fingertipPositon);
            _explorationFinger->getPosition(fingertipPositon);
            // Check if it is high enough
            if(!(fabs(fingertipPositon[2]  + 0.15) > 10.0/1000))
            {
                  _contactState = SET_WAYPOINT_GP;
            }

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
            //GPExplorationThread::moveToNewLocation();
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
            //TappingExplorationThread::finshExploration();
            break;
        }

    }


}

bool ExploreGPSurfaceThread::initialiseGP(yarp::sig::Vector startingPos, yarp::sig::Vector startingOrient, yarp::sig::Vector endingPos, yarp::sig::Vector endingOrient)
{
    //double xMin, xMax, yMin, yMax, zMin;
    int nSteps = 8;


    _surfaceModel->loadContactData("GP");
    _surfaceModel->setBoundingBox(nSteps, 0/1000);
    _surfaceModel->trainModel();
    _surfaceModel->updateSurfaceEstimate();
    _wayPointList.clear();
    _wayPointListComplete = false;

}

void ExploreGPSurfaceThread::maintainContact()
{

   /* if(!_wayPointListComplete)
    {

        Vector fingertipPositon;
        _objectFeatures->getIndexFingertipPosition(fingertipPositon);

        // Check if it is high enough
        if(fabs(fingertipPositon[2]  + 0.15) > 15.0/1000)
        {

        }
        _contactState = SET_WAYPOINT_GP;



    }
    else
    {
        moveArmUp();

        // Open the fingertip
        _objectFeatures->fingerMovePosition(11, 0);
        _objectFeatures->fingerMovePosition(12, 15);
        while (!_objectFeatures->checkOpenHandDone())
            ;

        // Go to the first position in the list
        Vector fingertipPosition;
        Vector indexFingerAngles;
        Vector desiredArmPos, desiredArmOrient;
        _objectFeatures->getStartingPose(desiredArmPos, desiredArmOrient);

        if(_wayPointList.empty())
            return;

        double offset = 3.0/1000;
        for (int i = 0; i < _wayPointList.size(); i++)
        {
            fingertipPosition = _wayPointList.at(i);
            cout << "DF: " << fingertipPosition.toString() << endl;
            // Get the current desired arm positionw with the new fingertip configuration
            _objectFeatures->indexFinger2ArmPosition(fingertipPosition, desiredArmPos);

            desiredArmPos[2] -= offset; // offset from the middle
            _robotCartesianController->goToPoseSync(desiredArmPos, desiredArmOrient);
           // _robotCartesianController->waitMotionDone(0.1, 2);



            //moveArmToWayPoint(desiredArmPos, desiredArmOrient);

            bool motionDone = false;
            while(!motionDone)
            {
                if(_objectFeatures->getContactForce() > 4)
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

            if(indexFingerAngles[1] > 0 )
            {
                cout << "No contact!" << endl;
                cout << "Angles: " << indexFingerAngles.toString() << endl;

                offset += 3.0/1000;
                i -= 1;
                continue;
            }
            else
            {
                offset = 6.0/1000;
            }


        }

        // Just to be safe
       //////////////////// move the arm up

        Vector startingPos, startingOrient;
        Vector armPos, orient;



        _objectFeatures->getArmPose(armPos, orient);
        _objectFeatures->getStartingPose(startingPos, startingOrient);


        _objectFeatures->indexFinger2ArmPosition(startingPos, desiredArmPos);
        armPos[2] = desiredArmPos[2]; // Move the fingertip up to avoid collisiont
        _objectFeatures->moveArmToPosition(armPos, orient);

        _contactState = STOP;


        return;
    }
    */

    // Get the fingertip position
    Vector indexFingerAngles;
    Vector fingertipPosition;
    _explorationFinger->getPosition(fingertipPosition);

    // set the current fingertip position as the next waypoint;
    Vector desiredArmPos, desiredArmOrient;
    _objectFeatures->getStartingPose(desiredArmPos, desiredArmOrient);

    // Open the fingertip
    _explorationFinger->setProximalAngle(0);
    _explorationFinger->setDistalAngle(10);
   // _objectFeatures->fingerMovePosition(11, 0);
    //_objectFeatures->fingerMovePosition(12, 10);
    while (!_explorationFinger->checkMotionDone())
        ;

    // Get the current desired arm positionw with the new fingertip configuration
    //_objectFeatures->indexFinger2ArmPosition(fingertipPosition, desiredArmPos);
     _explorationFinger->toArmPosition(fingertipPosition, desiredArmPos);

    //desiredArmPos[2] -= 5.0/1000; // offset from the middle
    //moveArmToWayPoint(desiredArmPos, desiredArmOrient);

    /////

    double offset = 3.0/1000;


    while(true){
        desiredArmPos[2] -= offset; // offset from the middle
        _robotHand->goToPoseSync(desiredArmPos, desiredArmOrient);
       // _robotCartesianController->waitMotionDone(0.1, 2);





        bool motionDone = false;
        while(!motionDone)
        {
            if(_explorationFinger->getContactForce() > 3)
            {
                cout  << "Abandoned motion due to force" << endl;
                _robotHand->stopControl();

                break;
            }

            _robotHand->checkMotionDone(&motionDone);
        }

        _explorationFinger->getPosition(fingertipPosition);
        cout << "AF: " << fingertipPosition.toString() << endl;



        _explorationFinger->getAngels(indexFingerAngles);
        //_objectFeatures->getIndexFingerAngles(indexFingerAngles);

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
  _robotHand->setAbduction(60);
   //_objectFeatures->fingerMovePosition(7, 60);
   yarp::os::Time::delay(2);
   _robotHand->setAbduction(0);
   //_objectFeatures->fingerMovePosition(7, 0);
   yarp::os::Time::delay(2);

    // Just to be safe
   //////////////////// move the arm up

    Vector startingPos, startingOrient;
    Vector armPos, orient;



   _robotHand->getPose(armPos, orient);
    _objectFeatures->getStartingPose(startingPos, startingOrient);


    _explorationFinger->toArmPosition(startingPos, desiredArmPos);
    //_objectFeatures->indexFinger2ArmPosition(startingPos, desiredArmPos);
    armPos[2] = desiredArmPos[2] + 10.0/1000; // Move the fingertip up to avoid collisiont
   _robotHand->goToPoseSync(armPos, orient, 10);


    _curProximal = 10;
    _curAbduction = 0;
    moveIndexFinger(_curProximal, _curAbduction);
   // _objectFeatures->setProximalAngle(10); // Needed by apporach object method



    _contactState = SET_WAYPOINT_GP;




    // I  should move the finger a little here

    //_contactState =  STOP; //SET_WAYPOINT_GP;*/
}

void ExploreGPSurfaceThread::moveArmToWayPoint(yarp::sig::Vector pos, yarp::sig::Vector orient)
{
    Vector indexFingerAngles;
    if(_robotHand->goToPoseSync(pos, orient))
    {
        bool motionDone = false;
        while(!motionDone)
        {
            if(_explorationFinger->getContactForce() > _forceThreshold)
            {
                cout  << "Abandoned motion due to force" << endl;
                _robotHand->stopControl();

                break;
            }

            _explorationFinger->getAngels(indexFingerAngles);
            //_objectFeatures->getIndexFingerAngles(indexFingerAngles);

            if(indexFingerAngles[1] < 3 )
            {
                cout << "Abandoned motion due to angles" << endl;
                _robotHand->stopControl();
                cout << "Angles: " << indexFingerAngles.toString() << endl;

                break;
            }
            _robotHand->checkMotionDone(&motionDone);
        }
    }
}
void ExploreGPSurfaceThread::setWayPoint_GP()
{
    // Use the GP model to calculate a new waypoint

    //Vector nextSamplingPos;
    Vector desiredArmOrient;
    Vector desiredArmPos;
    bool ret;

    ret = true;
    // I am not sure if I need to keep track of them anymore
    _curProximal = 10;
    //_curDistal = 90 - _curProximal;
    _curAbduction = 0;
    moveIndexFingerBlocking(_curProximal, _curAbduction, 40);
    // It also publishes it
   /* _objectFeatures->setProximalAngle(_curProximal);
    while(!_objectFeatures->checkOpenHandDone() && !isStopping())
   */     ;
    // Just to get the desired orientation
    _objectFeatures->getStartingPose(desiredArmPos, desiredArmOrient);


    // I have to make sure the new waypoint is valid
    if(_surfaceModel->getNextSamplingPosition(_nextSamplingPos))
    //if(_surfaceModel->getMaxEstimatePos(_nextSamplingPos))
    {

        //Get the current fingertip position
        //Vector tipPos, tipOrient;
        _explorationFinger->toArmPosition(_nextSamplingPos, desiredArmPos);

        //_objectFeatures->indexFinger2ArmPosition(_nextSamplingPos, desiredArmPos);
        _objectFeatures->setWayPointGP(desiredArmPos, desiredArmOrient);
    }
    else
    {
        _wayPointListComplete = true;
        _contactState = MAINTAIN_CONTACT;
    }


    _contactState = APPROACH_OBJECT;


}

} // end of namespace
