#include "tappingExplorationthread.h"
#include <iostream>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>
#include <cmath>

namespace objectExploration
{



using std::cout;
using std::endl;
using std::cerr;

using yarp::sig::Vector;



void TappingExplorationThread::run()
{

    Vector finger_pos, finger_orient;
    finger_pos.resize(3);
    finger_orient.resize(4);

    //State contactState = State::UNDEFINED;

    _contactState = APPROACH_OBJECT;


    while(!isStopping() && !(_contactState == STOP)) // Keep running this
    {




        cout << "Tapping away!" << endl;


        switch (_contactState)
        {

        case UNDEFINED:
            // This is the first round no approach has been made
            // Get the waypoint and set the state to approaching

            break;
        case  APPROACH_OBJECT:
            // Aproach and wait for contact
            // If contact set the state to contact
            // If reach the limit set the state to MOVELOCATION
            cout << "Contact state is: approach" << endl;
            approachObject();
            break;
        case CALCULATE_NEWWAYPONT:
             cout << "Contact state is: new waypoint" << endl;
            calculateNewWaypoint();
            break;
        case MAINTAIN_CONTACT:
             cout << "Contact state is: maintain contact" << endl;
            // Maintain contact for a couple of seconds
            // Then set the state to move location
            maintainContact();
            break;
        case MOVE_LOCATION:
             cout << "Contact state is: move location" << endl;
            // Calculate the next waypoint
            moveToNewLocation();
            //continue; // Just for now, before I clean the code
            break;
        case FINISHED:
             cout << "Contact state is: finished" << endl;
            cout << "Exploration completed" << endl;
            // I have to implement exit the thread procedure here
            finshExploration();
            break;
        }


    }


    // Make sure nothing is conrolling the finger
    _objectFeatures->writeToFingerController("stop");

    yarp::os::Time::delay(1);





}

void TappingExplorationThread::finshExploration()
{
    Vector starting_pos, starting_orient;
    starting_pos.resize(3);
    starting_orient.resize(4);

    // Open the hand
    _objectFeatures->openHand();

    // Wait for the hand to open;
    while(!_objectFeatures->checkOpenHandDone() && !isStopping())
        ;

    if(_objectFeatures->getStartingPose(starting_pos, starting_orient))
    {
        //
        _robotCartesianController->goToPoseSync(starting_pos, starting_orient);
        _robotCartesianController->waitMotionDone();
    }

    _contactState = STOP;

}

void TappingExplorationThread::maintainContact()
{
    _objectFeatures->writeToFingerController("task add ctrl 20");  // TODO: put it in the config file
    _objectFeatures->writeToFingerController("start");
    yarp::os::Time::delay(2);
    _contactState = MOVE_LOCATION;
}

void TappingExplorationThread::calculateNewWaypoint()
{

    Vector finger_pos, finger_orient;
    finger_pos.resize(3);
    finger_orient.resize(4);
    finger_pos.zero();
    finger_orient.zero();

    Vector px, ox;
    px.resize(3);
    px.zero();
    ox.resize(4);
    ox.zero();


    if(_objectFeatures->getWayPoint(px, ox))
    {
        // Stop the finger controller
        cout << "Stopping the finger controller...";
        _objectFeatures->writeToFingerController("stop");
        cout << "Done!" << endl;


        // Get the finger postion
        cout << "Reading the fingertip position...";
        _objectFeatures->getFingertipPose(finger_pos, finger_orient);
        cout << "Done!" << endl;
        cout << "Fingertip position: " << finger_pos.toString() << endl;

        //yarp::os::Time::delay(1);

        cout << "Opening the hand...";
        //Open the finger
        _objectFeatures->openHand();

        //Wait for the fingertip to open
        while(!_objectFeatures->checkOpenHandDone() && !isStopping())
            ;
        cout << "Done!" << endl;




        px[2] -= 0.02; // finger_pos[2]; // Hack, until I figure out why the finger position is not correct
        _objectFeatures->setWayPoint(px, ox);

        _contactState = APPROACH_OBJECT;

    }
    else
    {
        std::cerr << endl << "Got invalid waypoint" << endl;
    }




}

void TappingExplorationThread::approachObject()
{

    cout << "Approaching the object" << endl;

    /// Position the hand at the waypoint
    Vector px, ox;
    px.resize(3);
    px.zero();
    ox.resize(4);
    ox.zero();

    cout << "Getting the next waypoint...";
    if(_objectFeatures->getWayPoint(px, ox, false))
    {
        cout << "done!" << endl;
        cout << "Moving to the waypoint...";
        // Go to the wayPoint if only it is a valid wayPoint.
        if(_robotCartesianController->goToPoseSync(px, ox))
            _robotCartesianController->waitMotionDone(0.1, 2);
        cout << "done!" << endl;
    }else if(_contactState != UNDEFINED)
    {
        cout << "the Waypoint is invalid." << endl;
        _contactState = MOVE_LOCATION;
    }



    if(_contactState == APPROACH_OBJECT)
    {



        cout << "Approaching...";
        /// Tell the finger controller to approach the object
        _objectFeatures->writeToFingerController("task add appr");
        _objectFeatures->writeToFingerController("start");


        cout << "Waiting for contact ...";

        // Wait for a contact or the distal angle being beyond certain limit
        // bool inContact = true;
        while(_objectFeatures->getContactForce() < 3)
        {

            if(isStopping())
            {
                break;
            }
            else if(_objectFeatures->getProximalJointAngle() > 50) // TODO: implement a timeout
            {
                cout << "No contact detected." << endl;
                _contactState = CALCULATE_NEWWAYPONT;
                break;
            }
        }

        if(_contactState != CALCULATE_NEWWAYPONT)
        {
            cout << "We have detected contact" << endl;
            _contactState = MAINTAIN_CONTACT;
        }
    }

}

void TappingExplorationThread::moveToNewLocation()
{

    cout << "Calculating new waypoint" << endl;
    Vector starting_pos, starting_orient;
    Vector end_pos, end_orient;
    Vector wayPoint_pos, wayPoint_orient;

    starting_pos.resize(3);
    starting_orient.resize(4);

    end_pos.resize(3);
    end_orient.resize(4);

    wayPoint_pos.resize(3);
    wayPoint_orient.resize(4);

    if(!_objectFeatures->getStartingPose(starting_pos, starting_orient))
    {
        cerr << "Cannot set a new location. Starting point is invalid" << endl;
        return;
    }

    if(!_objectFeatures->getDesiredEndPose(end_pos, end_orient))
    {
        cerr << "Cannot set a new location. Desired end-point is invalid" << endl;
        return;
    }


    // Get the current wayPoint

    _objectFeatures->getWayPoint(wayPoint_pos, wayPoint_orient);

    if(wayPoint_pos[0] == 0)
    {
        cerr << "Cannot set new location, previous waypoint is invalid" << endl;
    }

    //////////////// Calculating a new waypoint for the travaersal /////////////////
    wayPoint_pos[1] += ((end_pos[1] - starting_pos[1]) * 0.9);
    wayPoint_pos[2] = starting_pos[2];

    _objectFeatures->setWayPoint(wayPoint_pos, wayPoint_orient);


    cout << "WayPoint: " << fabs(wayPoint_pos[1]) << " " << "EndPos: " << fabs(end_pos[1]) << endl;
    if(fabs(wayPoint_pos[1]) > fabs(end_pos[1]))
        _contactState = APPROACH_OBJECT;
    else
    {
        cout << "State set to finished" << endl;
        _contactState = FINISHED;
    }


}

bool TappingExplorationThread::threadInit()
{
    return true;
}


void TappingExplorationThread::threadRelease()
{

}

} // namespace objectExploration
