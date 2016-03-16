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
    _curDistal = 0;
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

        //_objectFeatures->updateContactState(_contactState);




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
    _wayPointList.clear();
    _wayPointListComplete = false;

}

void ExploreGPSurfaceThread::maintainContact()
{

    if(!_wayPointListComplete)
    {

        Vector fingertipPositon;
        _objectFeatures->getIndexFingertipPosition(fingertipPositon);
        // Hack for smoot motion
        fingertipPositon[0] = _nextSamplingPos[0];
        fingertipPositon[1] = _nextSamplingPos[1];
        _wayPointList.push_back(fingertipPositon);
        _contactState = SET_WAYPOINT_GP;
        return;
    }
    else
    {
        moveArmUp();

        // Open the fingertip
        _objectFeatures->fingerMovePosition(11, 0);
        _objectFeatures->fingerMovePosition(12, 10);
        while (!_objectFeatures->checkOpenHandDone())
            ;

        // Go to the first position in the list
        Vector fingertipPosition;
        Vector desiredArmPos, desiredArmOrient;
        _objectFeatures->getStartingPose(desiredArmPos, desiredArmOrient);

        if(_wayPointList.empty())
            return;

        for (int i = 0; i < _wayPointList.size(); i++)
        {
            fingertipPosition = _wayPointList.at(i);
            cout << "Fingertip: " << fingertipPosition.toString() << endl;
            // Get the current desired arm positionw with the new fingertip configuration
            _objectFeatures->indexFinger2ArmPosition(fingertipPosition, desiredArmPos);

            desiredArmPos[2] += 0.0/1000; // offset from the middle
            _robotCartesianController->goToPoseSync(desiredArmPos, desiredArmOrient);
            _robotCartesianController->waitMotionDone(0.1, 2);
            //moveArmToWayPoint(desiredArmPos, desiredArmOrient);

           /* bool motionDone = false;
            while(!motionDone)
            {
                if(_objectFeatures->getContactForce() > 4)
                {
                    cout  << "Abandoned motion due to force" << endl;
                    _robotCartesianController->stopControl();

                    break;
                }

            }

            */
        }

        // Just to be safe
       //////////////////// move the arm up
       ///
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
    // Get the fingertip position
  /*  Vector fingertipPositon;
    _objectFeatures->getIndexFingertipPosition(fingertipPositon);

    // set the current fingertip position as the next waypoint;
    Vector desiredArmPos, desiredArmOrient;
    _objectFeatures->getStartingPose(desiredArmPos, desiredArmOrient);

    // Open the fingertip

    _objectFeatures->fingerMovePosition(11, 0);
    _objectFeatures->fingerMovePosition(12, 10);
    while (!_objectFeatures->checkOpenHandDone())
        ;

    // Get the current desired arm positionw with the new fingertip configuration
    _objectFeatures->indexFinger2ArmPosition(fingertipPositon, desiredArmPos);

    desiredArmPos[2] -= 5.0/1000; // offset from the middle
    moveArmToWayPoint(desiredArmPos, desiredArmOrient);

    // I  should move the inger a little here

    _contactState =  STOP; //SET_WAYPOINT_GP;*/
}

void ExploreGPSurfaceThread::moveArmToWayPoint(yarp::sig::Vector pos, yarp::sig::Vector orient)
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

            if(indexFingerAngles[1] < 3 )
            {
                cout << "Abandoned motion due to angles" << endl;
                _robotCartesianController->stopControl();
                cout << "Angles: " << indexFingerAngles.toString() << endl;

                break;
            }
            _robotCartesianController->checkMotionDone(&motionDone);
        }
    }
}
void ExploreGPSurfaceThread::setWayPoint_GP()
{
    // Use the GP model to calculate a new waypoint

    //Vector nextSamplingPos;
    Vector orient;
    Vector armPos;
    bool ret;

    ret = true;
    _curProximal = 10;
    _curDistal = 90 - _curProximal;
    _objectFeatures->setProximalAngle(_curProximal);

    //_objectFeatures->prepHand();
    while(!_objectFeatures->checkOpenHandDone() && !isStopping())
        ;
    // Get the valid point from object features
    _objectFeatures->getWayPoint(armPos, orient, false);

    // I have to make sure the new waypoint is valid
    if(_surfaceModel->getNextSamplingPosition(_nextSamplingPos))
    {
        //Get the current fingertip position
        //Vector tipPos, tipOrient;
        _objectFeatures->indexFinger2ArmPosition(_nextSamplingPos, armPos);
        _objectFeatures->setWayPointGP(armPos, orient);
    }
    else
    {
        _wayPointListComplete = true;
    }


    // Get the position of the hand
    // This cannot be done in parallel with indexFinger2ArmPosition calcuation

    //moveArmUp();

    if(ret)
        _contactState = APPROACH_OBJECT;
    else
        _contactState = STOP;

    //return ret;

}

} // end of namespace
