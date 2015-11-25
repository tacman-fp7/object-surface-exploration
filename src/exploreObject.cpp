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




ExploreObject::ExploreObject(yarp::os::ResourceFinder& rf)
{

    bool failed = false;
    _exploreObjectOnOff = true;
    _stopModule = false;
    _rf = rf;

    _maintainContactThread = NULL;


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

    if(_maintainContactThread != NULL)
    {

        delete(_maintainContactThread);
        _maintainContactThread = NULL;
    }
    //cout << "Here2" << endl;
    if(_objectFeaturesThread != NULL)
    {

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

    if(_exploreObjectOnOff)
    {
        //TODO: do some checks if the thread is running on so on
        // First step is to reach the pre-contact location
        if(!this->goToStartingPose())
            ret = false;
        _armCartesianController->waitMotionDone();

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
       // if(!this->goToHomePose())
       //     ret = false;

        if(!this->goToStartingPose())
            ret = false;

        //_maintainContactThread->stop();

        _exploreObjectThread->stop();



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

    _dbgtag = "exploreObject.cpp: ";



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

    cout << _dbgtag << "Device options: " << deviceOptions.toString() << endl;

    if(!_deviceController.open(deviceOptions))
    {
        cerr << _dbgtag << "Failed to open the device: " << systemParameters.getControllerType() << endl;
        return false;
    }

    // Open a Cartesian controller

    if(!_deviceController.view(_armCartesianController))
    {
        cerr << _dbgtag << "Failed to get a Cartesian view" << endl;
        return false;
    }

    // Remember the contorller context ID, restore when closing the port
    _armCartesianController->storeContext(&_cartCtrlStartupIDstartupID);

    // Set the trajectory time
    _armCartesianController->setTrajTime(systemParameters.getTrajectoryTime());


    //////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////        Joint Control                     /////////////////
    //////////////////////////////////////////////////////////////////////////////////////

    yarp::os::Property optionsJnt;
    optionsJnt.put("device", "remote_controlboard");
    optionsJnt.put("local", "/" + moduleName + "/" + systemParameters.getArm() + "_arm/joint");                 //local port names
    optionsJnt.put("remote", "/" + systemParameters.getRobotName()
                    + "/" + systemParameters.getArm() + "_arm");

    //"/icubSim/right_arm");

    cout << _dbgtag << "Device options: " << optionsJnt.toString() << endl;

    if(!_deviceController_joint.open(optionsJnt))
    {
        cerr << _dbgtag << "Failed to open the device: " << "urgh" << endl; //systemParameters.getControllerType() << endl;
        return false;
    }


    // Open an encoder view
    if(!_deviceController_joint.view(_armEncoders))
    {
        cerr << _dbgtag << "Failed to open Encoder view" << endl;
    }

    //_armCartesianController->getPose()
    ////////////////////////////////////////////////////////////////////////////////////////
    ////////// Setting up the tactile data reading thread //////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////


    _objectFeaturesThread->setArmController_cart(_armCartesianController);
    _objectFeaturesThread->setArmController_jnt(_armEncoders);

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



