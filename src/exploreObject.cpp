#include <exploreObject.h>
#include <yarp/sig/Vector.h>
#include <signal.h>
#include <yarp/os/Bottle.h>
#include <tappingExplorationthread.h>


namespace objectExploration {



using std::cout;
using std::cerr;
using std::endl;
using yarp::os::Value;


bool ExploreObject::openHand()
{

    return _objectFeaturesThread->openHand();
    while (!_objectFeaturesThread->checkOpenHandDone() && !isStopping())
        ;
}

bool ExploreObject::prepHand()
{
    return _objectFeaturesThread->prepHand();
    while (!_objectFeaturesThread->checkOpenHandDone() && !isStopping())
        ;
}

bool ExploreObject::calibrateHand()
{
    _objectFeaturesThread->calibrateHand();

    return true;
}

bool ExploreObject::fingerSetAngle(const double angle)
{

    Vector finger_pos;

    //_objectFeaturesThread->getIndexFingertipPosition(finger_pos);

    Vector pos, orient;


    _objectFeaturesThread->getFingertipPose(pos, orient);

    cout << "Finger: " << finger_pos.toString() << endl;
return _objectFeaturesThread->setProximalAngle(angle);
}

ExploreObject::ExploreObject(yarp::os::ResourceFinder& rf)
{

    bool failed = false;
    _exploreObjectOnOff = true;
    _exploreObjectValid = true; // Assume it is true, set it to false when something fails
    _stopModule = false;
    _rf = rf;

    _maintainContactThread = NULL;
    _exploreObjectThread = NULL;

    //// TODO: I save system parameters here that I use in this module.
    /// This is not a good idea. I should change it.
    int readTactilePeriod;
    Bottle& explorationParameters = _rf.findGroup("ExplorationParameters");
    if(!explorationParameters.isNull())
    {
        readTactilePeriod = explorationParameters.check("readTactilePeriod", Value(20)).asInt();
    }

    _objectFeaturesThread = new ObjectFeaturesThread(readTactilePeriod,  rf);

}

ExploreObject::~ExploreObject()
{

    //cout << "Here" << endl;

    if(_exploreObjectThread != NULL)
    {
        if(_exploreObjectThread->isRunning())
            _exploreObjectThread->stop();

        delete(_exploreObjectThread);
        _exploreObjectThread = NULL;
    }
    if(_maintainContactThread != NULL)
    {

        delete(_maintainContactThread);
        _maintainContactThread = NULL;
    }
    //cout << "Here2" << endl;
    if(_objectFeaturesThread != NULL)
    {

        if(_objectFeaturesThread->isRunning())
            _objectFeaturesThread->askToStop();

        delete(_objectFeaturesThread);
        _objectFeaturesThread = NULL;
    }
    //cout << "Here3" << endl;


}

bool ExploreObject::goToStartingPose()
{
    Vector pos, orient;
    pos.resize(3);
    orient.resize(4);
    if(_objectFeaturesThread->getStartingPose(pos, orient))
    {

/*        // Quick test
       double armJoints[16];
       memset(armJoints, 0, sizeof(armJoints));
       armJoints[0] = -20;
       armJoints[1] = 29;
       armJoints[2] = 42;
       armJoints[3] = 46;
       armJoints[4] = 57;
       armJoints[5] = -14;
       armJoints[6] = -7;
       armJoints[7] = 39;
       armJoints[8] = 11;

       _armJointPositionController->positionMove(armJoints);

*/
        _armCartesianController->goToPoseSync(pos, orient);
        return true;
    }

    return false;

}



bool ExploreObject::goToHomePose()
{
    Vector pos, orient;
    pos.resize(3); // x,y,z position
    orient.resize(4); // x,y,z,w prientation

    if(_objectFeaturesThread->getHomePose(pos, orient))
    {
        _armCartesianController->goToPoseSync(pos, orient);
        return true;
    }
    return false;
}

bool ExploreObject::goToEndPose()
{
    Vector pos, orient;
    pos.resize(3); // x,y,z position
    orient.resize(4); // x,y,z,w prientation
    if(_objectFeaturesThread->getDesiredEndPose(pos, orient))
    {
        _armCartesianController->goToPoseSync(pos, orient);
        return true;
    }
    return false;


}

bool ExploreObject::setStartingPose()
{
    Vector pos, orient;
    pos.resize(3); // x,y,z position
    orient.resize(4); // x,y,z,w prientation
    _armCartesianController->getPose(pos, orient);
    _objectFeaturesThread->setStartingPose(pos, orient);
    return true;
}

bool ExploreObject::setHomePose()
{
    Vector pos, orient;
    pos.resize(3); // x,y,z position
    orient.resize(4); // x,y,z,w prientation
    _armCartesianController->getPose(pos, orient);
    _objectFeaturesThread->setHomePose(pos, orient);
    return true;
}

bool ExploreObject::setEndPose()
{
    Vector pos, orient;
    pos.resize(3); // x,y,z position
    orient.resize(4); // x,y,z,w prientation
    _armCartesianController->getPose(pos, orient);
    _objectFeaturesThread->setEndPose(pos, orient);
    return true;
}

// bool ExploreObject::updateHomePose()
// {
//    Vector pos, orient;
//   pos.resize(3); // x,y,z position 
//   orient.resize(4); // x,y,z,w prientation
//   _armCartesianController->getPose(pos, orient);
//   _approachObjectCntrl->updateHomePose(pos, orient);
// return true;
// }

bool ExploreObject::startExploring()
{
    bool ret = true;

    cout << "Explore object starting" << endl;

    if(!_exploreObjectValid || !_objectFeaturesThread->isExplorationValid())
    {
        cerr << _dbgtag << "Cannot start exploring, one or more of the dependencies have not been met" << endl;
        return false;
    }

    if(_exploreObjectOnOff)
    {
        //TODO: do some checks if the thread is running on so on
        // First step is to reach the pre-contact location
        prepHand();
        if(!this->goToStartingPose())
            ret = false;
        _armCartesianController->waitMotionDone(0.1, 20);

        // Ge the current position of the arm.
        Vector pos, orient;
        pos.resize(3);
        orient.resize(4);
        if(!_objectFeaturesThread->getArmPose(pos, orient))
        {
            cerr << _dbgtag << "Could not read the arm position, cannot start exploration" << endl;
            return false;
        }

        // Setting the way point to the start of the exploration
        _objectFeaturesThread->setWayPoint(pos, orient);


        // Then explore the object
        //if(!_maintainContactThread->start())
        //    ret = false;

        if(!_exploreObjectThread->start())
            ret = false;

        cout << "Exoploring the object\n" << endl;
        _exploreObjectOnOff = false;

    }
    else{

        cout << "Warning! Already exploring." << endl;
    }

    return ret;
}

bool ExploreObject::stopExploring()
{

    bool ret = true;

    if(_exploreObjectOnOff)
    {
        cout << "Warning! Nothing to stop, are you sure." << endl;
    }
    else
    {


        _exploreObjectThread->stop();

        cout << "stopped" << endl;
       // if(!this->goToHomePose())
       //     ret = false;



       /* if(!goToStartingPose())
            ret = false;

        _armCartesianController->waitMotionDone(0.1, 20);

        // Try to go to home pose if available;
        goToHomePose();

        _armCartesianController->waitMotionDone(0.1, 20);


         openHand();

         */
        //_maintainContactThread->stop();





        cout << "Stopped the exploration" << endl;
        _exploreObjectOnOff = true;
    }

    return ret;

}



////////////////// RF module implementation //////////////////


// Attach the port as a server
bool ExploreObject::attach ( yarp::os::Port& source )
{

    return this->yarp().attachAsServer(source);
}

bool ExploreObject::configure(yarp::os::ResourceFinder& rf )
{


    bool ret = true;

    // Check if in the config file we have a name for the server
    string moduleName = rf.check("moduleName", Value("object-exploration-server"),
                                 "module name (string)").asString().c_str();

    setName(moduleName.c_str());

    _dbgtag = "\n\nexploreObject.cpp: ";



    ObjectFeaturesThread& systemParameters = *_objectFeaturesThread; // Just for better naming


    ///////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////// Configure the controller //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////

    yarp::os::Property deviceOptions;
    deviceOptions.put("device", systemParameters.getControllerType());
    deviceOptions.put("local", "/" + moduleName
                      + "/" + systemParameters.getControllerName() + "/" + systemParameters.getArm() + "_arm");
    deviceOptions.put("remote", "/" + systemParameters.getRobotName()
                      + "/" + systemParameters.getControllerName() + "/" + systemParameters.getArm() + "_arm");

    cout << "Device options: " << deviceOptions.toString() << endl;

    if(!_deviceController.open(deviceOptions))
    {
        cerr << _dbgtag << "Failed to open the device: " << systemParameters.getControllerType() << endl;
        // cannot explore
        _exploreObjectValid = false;
        return false;
    }

    // Open a Cartesian controller

    if(!_deviceController.view(_armCartesianController))
    {
        cerr << _dbgtag << "Failed to get a Cartesian view" << endl;
        // Cannot explore,
        _exploreObjectValid = false;
        return false;
    }

    // Remember the contorller context ID, restore when closing the port
    _armCartesianController->storeContext(&_cartCtrlStartupIDstartupID);

    // Set the trajectory time
    _armCartesianController->setTrajTime(systemParameters.getTrajectoryTime());

    // Enable the torso movement

    Vector curDof;
    _armCartesianController->getDOF(curDof);
    cout<<"["<<curDof.toString()<<"]"<<endl;  // [0 0 0 1 1 1 1 1 1 1] will be printed out
    Vector newDof(3);
    newDof[0]=1;    // torso pitch: 1 => enable
    newDof[1]=2;    // torso roll:  2 => skip
    newDof[2]=1;    // torso yaw:   1 => enable
    _armCartesianController->setDOF(newDof,curDof);
    cout<<"["<<curDof.toString()<<"]"<<endl;  // [1 0 1 1 1 1 1 1 1 1] will be printed out

    //_armCartesianController->setPosePriority("orientation");


    //////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////        Joint Control                     /////////////////
    //////////////////////////////////////////////////////////////////////////////////////

    yarp::os::Property optionsJnt;
    optionsJnt.put("device", "remote_controlboard");
    optionsJnt.put("local", "/" + moduleName + "/" + systemParameters.getArm() + "_arm/joint");                 //local port names
    optionsJnt.put("remote", "/" + systemParameters.getRobotName()
                    + "/" + systemParameters.getArm() + "_arm");



    cout << "Device options: " << optionsJnt.toString() << endl;

    if(!_deviceController_joint.open(optionsJnt))
    {
        cerr << _dbgtag << "Failed to open the device: " << "urgh" << endl; //systemParameters.getControllerType() << endl;
        // Cannot explore
        _exploreObjectValid = false;
        return false;
    }

    //open an armcontroller_mode view
    if(!_deviceController_joint.view(_armController_mode))
    {
        cerr << _dbgtag << "Failed to open control mode view" << endl;
        // Cannot explore
        _exploreObjectValid = false;
        return false;
    }

    // Open an encoder view
    if(!_deviceController_joint.view(_armEncoders))
    {
        cerr << _dbgtag << "Failed to open Encoder view" << endl;
        // Cannot explore
        _exploreObjectValid = false;
        return false;
    }

    if(!_deviceController_joint.view(_armJointPositionController))
    {
        cerr << _dbgtag << "Failed to open joint position controller view" << endl;
        _exploreObjectValid = false;
        return false;
    }

    int armJointsNum;
    _armJointPositionController->getAxes(&armJointsNum);
    // Set reference speeds
    std::vector<double> refSpeeds(armJointsNum, 0);
    _armJointPositionController->getRefSpeeds(&refSpeeds[0]);
    for (int i = 11; i < 15; ++i) {
        refSpeeds[i] = 50;
    }
    _armJointPositionController->setRefSpeeds(&refSpeeds[0]);

    //_armCartesianController->getPose()
    ////////////////////////////////////////////////////////////////////////////////////////
    ////////// Setting up the tactile data reading thread //////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////


    _objectFeaturesThread->setArmController_cart(_armCartesianController);
    _objectFeaturesThread->setArmController_jnt(_armEncoders, _armJointPositionController);
    _objectFeaturesThread->setArmController_mode(_armController_mode);

    _objectFeaturesThread->start();

    ////////////////////////////////////////////////////////////////////////////////////////
    ////////// Setting up the MaintainContactThread ////////////////////////////////////////
    ////////// at the meoment it is achieved using Massimo's code //////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////
    _maintainContactThread = new MaintainContactThread(systemParameters.getMaintainContactPeriod(),
                                                       _objectFeaturesThread);
    _maintainContactThread->setDesiredForce(systemParameters.getDesiredForce());

    /////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////// Setting up the exploration strategy thread ///////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////
    _exploreObjectThread = new TappingExplorationThread(systemParameters.getExplorationThreadPeriod(),
                                                       _armCartesianController,_objectFeaturesThread);





    std::string portName= "/";
    portName+= getName() +"/rpc:i";
    if (!_robotControl_port.open(portName.c_str())) {
        cerr << _dbgtag << ": Unable to open port " << portName << endl;
        return false;
    }


    this->attach(_robotControl_port);

    setHomePose();

    return ret;
}


bool ExploreObject::close()
{


    ////
    if(_objectFeaturesThread != NULL)
        _objectFeaturesThread->askToStop();




    ////



    // Close neatly, this function is called when Ctl+C is registered
    /// To be safe, stop the control
    _armCartesianController->stopControl();

    /// Store the old context to return the robot to the settings before
    /// this module
    _armCartesianController->restoreContext(_cartCtrlStartupIDstartupID);
    _deviceController.close();


    _deviceController_joint.close();

    _robotControl_port.close();
                                  // Close the device controller


    return true;
}

bool ExploreObject::quit()
{
    _stopModule = true;
    return true;
}


bool ExploreObject::updateModule()
{

    // Put a repetitive task here that will be run every getPeriod() time

    return (!_stopModule);

}




} // namespace objectExploration



