#include <exploreObject.h>
#include <yarp/sig/Vector.h>
#include <signal.h>
#include <yarp/os/Bottle.h>
#include <tappingExplorationThread.h>


namespace objectExploration {



using std::cout;
using std::cerr;
using std::endl;
using yarp::os::Value;

bool ExploreObject::validatePositionsEnable(){
    _exploreObjectGP_thread->enableValidatePositions();
    return true;
}

bool ExploreObject::validatePositionsDisable(){
    _exploreObjectGP_thread->disaleValidatePositions();
    return true;
}

bool ExploreObject::nRepeatsSet(const int32_t nRepeats){
    _exploreObjectGP_thread->setNRepeats(nRepeats);
    return true;
}

bool ExploreObject::refineModelEnable(){
    _exploreObjectGP_thread->enableRefiningModel();
    return true;
}

bool ExploreObject::refineModelDisable(){
    _exploreObjectGP_thread->disableRefiningModel();
    return true;
}

bool ExploreObject::openHand(){
    return _robotHand->open();
}

bool ExploreObject::prepHand(){
    //return _robotHand->prepare();
    cout << "Force: " << _explorationFinger->getContactForce() << endl;
    Vector cop;
    _explorationFinger->getContactCoP(cop);
    cout << "CoP: " << cop.toString() << endl;
}

bool ExploreObject::calibrateHand(){
    _robotHand->calibrate();
    return true;
}

bool ExploreObject::fingerSetAngle(const double angle){
    Vector finger_pos;
    bool ret;

    ret = _explorationFinger->setSynchroProximalAngle(angle);
    while(!_explorationFinger->checkMotionDone())
        ;

    _explorationFinger->getPosition(finger_pos);
    cout << "Finger pos: " << finger_pos.toString() << endl;
    _explorationFinger->getAngels(finger_pos);
    cout << "Finger ang: " << finger_pos.toString() << endl;


    return ret;

}

ExploreObject::ExploreObject(yarp::os::ResourceFinder& rf)
{

    // bool failed = false;
    _exploreObjectOnOff = true;
    _exploreObjectValid = true; // Assume it is true, set it to false when something fails
    _stopModule = false;
    _rf = rf;

    //    _maintainContactThread = NULL;
    //_contactSafetyThread = NULL;
    _exploreObjectThread = NULL;
    _exploreObjectGP_thread = NULL;
    _exploreGPSurface_thread = NULL;

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

    if(_exploreObjectGP_thread != NULL)
    {
        if(_exploreObjectGP_thread->isRunning())
            _exploreObjectGP_thread->stop();
        delete(_exploreObjectGP_thread);
        _exploreObjectGP_thread = NULL;
    }

    if(_exploreGPSurface_thread != NULL)
    {
        if(_exploreGPSurface_thread->isRunning())
            _exploreGPSurface_thread->stop();
        delete(_exploreGPSurface_thread);
        _exploreGPSurface_thread = NULL;
    }
    /*if(_contactSafetyThread != NULL)
    {
        if(_contactSafetyThread->isRunning())
            _contactSafetyThread->stop();
        delete(ContactSafetyThread);
        _contactSafetyThread = NULL;

    }*/

    if(_objectFeaturesThread != NULL)
    {

        if(_objectFeaturesThread->isRunning())
            _objectFeaturesThread->askToStop();

        delete(_objectFeaturesThread);
        _objectFeaturesThread = NULL;
    }
    //cout << "Here3" << endl;


}

bool ExploreObject::goToStartingPose(){
    return _robotHand->goToStartingPose();
}





bool ExploreObject::goToEndPose(){
    return _robotHand->goToEndPose();
}

bool ExploreObject::setStartingPose()
{
    Vector pos, orient;
    pos.resize(3); // x,y,z position
    orient.resize(4); // x,y,z,w prientation
    //_armCartesianController->getPose(pos, orient);
    _robotHand->getPose(pos, orient);
    _objectFeaturesThread->setStartingPose(pos, orient);
    return true;
}

/*bool ExploreObject::setHomePose()
{
    Vector pos, orient;
    pos.resize(3); // x,y,z position
    orient.resize(4); // x,y,z,w prientation
    //_robotHand->getPose(pos, orient);
    //_objectFeaturesThread->setHomePose(pos, orient);
    return true;
}*/

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

bool ExploreObject::exploreGPSurface(const std::string& objectName)
{

    bool ret = true;
    cout << "Exploring GP generated surface: " << objectName << "." << endl;

    if(!_exploreObjectValid || !_objectFeaturesThread->isExplorationValid())
    {
        cerr << _dbgtag << "Cannot start exploring, one or more of the dependencies have not been met" << endl;
        return false;
    }


    if(_exploreObjectOnOff)
    {
        _robotHand->prepare();

        //prepHand();
        if(!this->goToStartingPose())
            ret = false;
        _armCartesianController->waitMotionDone(0.1, 5);

        // Ge the current position of the arm.
        Vector pos, orient;
        if(!_robotHand->getPose(pos, orient))
        {
            cerr << _dbgtag << "Could not read the arm position, cannot start exploration" << endl;
            return false;
        }


        // Setting the way point to the start of the exploration
        if(!_objectFeaturesThread->setWayPoint(pos, orient))
        {
            cerr << _dbgtag << "Failed to set the initial waypoint. Aborting exploration!" << endl;
            return false;
        }


        if(!_exploreGPSurface_thread->start())
            ret = false;

        _exploreObjectOnOff = false;

        cout << "Exoploring the GP Surface\n" << endl;

    }
    else{

        cout << "Warning! Already exploring." << endl;
    }



    return ret;



}


bool ExploreObject::startExploringGP()
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
        prepHand();
        if(!this->goToStartingPose())
            ret = false;
        _armCartesianController->waitMotionDone(0.1, 20);

        // Ge the current position of the arm.
        Vector pos, orient;
        pos.resize(3);
        orient.resize(4);
        if(!_robotHand->getPose(pos, orient))
        {
            cerr << _dbgtag << "Could not read the arm position, cannot start exploration" << endl;
            return false;
        }


        // Setting the way point to the start of the exploration
        if(!_objectFeaturesThread->setWayPoint(pos, orient))
        {
            cerr << _dbgtag << "Failed to set the initial waypoint. Aborting exploration!" << endl;
            return false;
        }

        /* Vector startingPos, endingPos, startingOrient, endingOrient;
        _objectFeaturesThread->getStartingPose(startingPos, startingOrient);
        _objectFeaturesThread->getEndingPose(endingPos, endingOrient);
        _exploreObjectGP_thread->initialiseGP(startingPos, startingOrient,
                                              endingPos, endingOrient);
        //_objectFeaturesThread->prepGP();

        */

        if(!_exploreObjectGP_thread->start())
            ret = false;

        _exploreObjectOnOff = false;

        cout << "Exoploring the object using GP\n" << endl;

    }
    else{

        cout << "Warning! Already exploring." << endl;
    }



    return ret;
}

bool ExploreObject::enableSurfaceSampling()
{
    if(_exploreObjectGP_thread != NULL)
    {
        _exploreObjectGP_thread->enableSurfaceSampling();
    }

    return true;
}

bool ExploreObject::disableSurfaceSampling()
{

    if(_exploreObjectGP_thread != NULL)
    {
        _exploreObjectGP_thread->disableSurfaceSampling();
    }
    return true;
}

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
        if(!_robotHand->getPose(pos, orient))
        {
            cerr << _dbgtag << "Could not read the arm position, cannot start exploration" << endl;
            return false;
        }

        // Setting the way point to the start of the exploration
        if(!_objectFeaturesThread->setWayPoint(pos, orient))
        {
            cerr << _dbgtag << "Failed to set the initial waypoint. Aborting exploration!" << endl;
            return false;
        }


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



        // 0 is invalid state!
        _objectFeaturesThread->updateContactState(0);
        if(_exploreObjectThread->isRunning())
            _exploreObjectThread->stop();
        if(_exploreObjectGP_thread->isRunning())
            _exploreObjectGP_thread->stop();
        if(_exploreGPSurface_thread->isRunning())
            _exploreGPSurface_thread->stop();


        cout << "stopped" << endl;

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

    Bottle robotParameters = rf.findGroup("RobotParameters");
    string robotName = robotParameters.check("robotName", Value("error")).asString();

    if(robotName.compare("icub") == 0){
        _robotHand = new icubHand(rf);
    }
    else if(robotName.compare("icubSim") == 0){
        _robotHand = new SimHand(rf);
    }
    else{
        return false;
    }
    _explorationFinger = _robotHand->getIndexFinger();

    // Check if in the config file we have a name for the server
    string moduleName = rf.check("moduleName", Value("object-exploration-server"),
                                 "module name (string)").asString().c_str();

    setName(moduleName.c_str());

    _dbgtag = "\n\nexploreObject.cpp: ";



    ObjectFeaturesThread& systemParameters = *_objectFeaturesThread; // Just for better naming



    ////////////////////////////////////////////////////////////////////////////////////////
    ////////// Setting up the tactile data reading thread //////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////


    //_objectFeaturesThread->setArmController_cart(_armCartesianController);
    //_objectFeaturesThread->setArmController_jnt(_armEncoders, _armJointPositionController);
    //_objectFeaturesThread->setArmController_mode(_armController_mode);

    //_objectFeaturesThread->start();




    /////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////// Setting up the exploration strategy thread ///////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////
    _exploreObjectThread = new TappingExplorationThread(systemParameters.getExplorationThreadPeriod(),
                                                        _robotHand, _explorationFinger,_objectFeaturesThread);


    _exploreObjectGP_thread = new GPExplorationThread(systemParameters.getExplorationThreadPeriod(),
                                                      _robotHand, _explorationFinger, _objectFeaturesThread);

    _exploreGPSurface_thread = new ExploreGPSurfaceThread(systemParameters.getExplorationThreadPeriod(),
                                                          _robotHand, _explorationFinger, _objectFeaturesThread);


    std::string portName= "/";
    portName+= getName() +"/rpc:i";
    if (!_robotControl_port.open(portName.c_str())) {
        cerr << _dbgtag << ": Unable to open port " << portName << endl;
        return false;
    }


    this->attach(_robotControl_port);

    //setHomePose();

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



