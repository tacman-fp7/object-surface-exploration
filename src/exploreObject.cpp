#include <exploreObject.h>
#include <yarp/sig/Vector.h>
#include <signal.h>
#include <yarp/os/Bottle.h>
#include <tappingExplorationThread.h>
#include <gridExplorationThread.h>
#include <fstream>
#include <yarp/os/Time.h>

namespace objectExploration {



using std::cout;
using std::cerr;
using std::endl;
using yarp::os::Value;

bool ExploreObject::multiFinger(const double angle){


    Finger *indexFinger = _robotHand->getIndexFinger();
    Finger *middleFinger = _robotHand->getMiddleFinger();

    yarp::sig::Vector position;
    for (int i = 1; i < 200; i++ ){
    position.clear();
    indexFinger->getPosition(position);
    _indexFingertipLog << position[0] << ", " << position[1] << ", " << position[2] << endl;
    position.clear();
    indexFinger->getPositionHandFrame(position);
    _indexFingerHandLog << position[0] << ", " << position[1] << ", " << position[2] << endl;

    position.clear();
    //indexFinger->getPositionHandFrameCorrected(position);
    indexFinger->getPositionCorrected(position);
    _indexCorrectedLog <<  position[0] << ", " << position[1] << ", " << position[2] << endl;

    position.clear();
    middleFinger->getPosition(position);
    _middleFingertipLog << position[0] << ", " << position[1] << ", " << position[2] << endl;

    position.clear();
    middleFinger->getPositionHandFrame(position);
    _middleFingerHandLog << position[0] << ", " << position[1] << ", " << position[2] << endl;

    Vector orient;
    position.clear();
    _robotHand->getPose(position, orient);
    _handPoseLog << position[0] << ", " << position[1] << ", " << position[2] << ", ";
    _handPoseLog << orient[0] << ", " << orient[1] << ", " << orient[2] << ", " << orient[3] << endl;
    yarp::os::Time::delay(0.1);
    }
return true;

}

/*bool ExploreObject::multiFinger(const double angle){
    //_robotHand->multiContact(angle);



    Finger *indexFinger = _robotHand->getIndexFinger();
    Finger *middleFinger = _robotHand->getMiddleFinger();

    for (int i = 20; i <= 50; i++){
        middleFinger->setSynchroProximalAngle(i);
        while(!middleFinger->checkMotionDone())
            ;


        //indexFinger->setSynchroProximalAngle(i);
        //while(!indexFinger->checkMotionDone())
        //    ;


        yarp::sig::Vector position;
        position.clear();
        indexFinger->getPosition(position);
        _indexFingertipLog << position[0] << ", " << position[1] << ", " << position[2] << endl;
        position.clear();
        indexFinger->getPositionHandFrame(position);
        _indexFingerHandLog << position[0] << ", " << position[1] << ", " << position[2] << endl;

        position.clear();
        //indexFinger->getPositionHandFrameCorrected(position);
        indexFinger->getPositionCorrected(position);
        _indexCorrectedLog <<  position[0] << ", " << position[1] << ", " << position[2] << endl;

        position.clear();
        middleFinger->getPosition(position);
        _middleFingertipLog << position[0] << ", " << position[1] << ", " << position[2] << endl;

        position.clear();
        middleFinger->getPositionHandFrame(position);
        _middleFingerHandLog << position[0] << ", " << position[1] << ", " << position[2] << endl;

        Vector orient;
        position.clear();
        _robotHand->getPose(position, orient);
        _handPoseLog << position[0] << ", " << position[1] << ", " << position[2] << ", ";
        _handPoseLog << orient[0] << ", " << orient[1] << ", " << orient[2] << ", " << orient[3] << endl;

    }

    for (int i = 50; i >= 20; i--){
        middleFinger->setSynchroProximalAngle(i);
        while(!middleFinger->checkMotionDone())
            ;


        //indexFinger->setSynchroProximalAngle(i);
        //while(!indexFinger->checkMotionDone())
        //    ;


        yarp::sig::Vector position;
        position.clear();
        indexFinger->getPosition(position);
        _indexFingertipLog << position[0] << ", " << position[1] << ", " << position[2] << endl;
        position.clear();
        indexFinger->getPositionHandFrame(position);
        _indexFingerHandLog << position[0] << ", " << position[1] << ", " << position[2] << endl;

        position.clear();
        //indexFinger->getPositionHandFrameCorrected(position);
        indexFinger->getPositionCorrected(position);
        _indexCorrectedLog <<  position[0] << ", " << position[1] << ", " << position[2] << endl;


        position.clear();
        middleFinger->getPosition(position);
        _middleFingertipLog << position[0] << ", " << position[1] << ", " << position[2] << endl;

        position.clear();
        middleFinger->getPositionHandFrame(position);
        _middleFingerHandLog << position[0] << ", " << position[1] << ", " << position[2] << endl;

        Vector orient;
        position.clear();
        _robotHand->getPose(position, orient);
        _handPoseLog << position[0] << ", " << position[1] << ", " << position[2] << ", ";
        _handPoseLog << orient[0] << ", " << orient[1] << ", " << orient[2] << ", " << orient[3] << endl;


    }


return true;

}*/

/*
bool ExploreObject::multiFinger(const double angle){
    indexFinger->setSynchroProximalAngle(angle);
    indexFinger->getPosition(position);
    cout << "I Finger: " << position.toString() << endl;


    MiddleFinger->setSynchroProximalAngle(angle);
    MiddleFinger->getPosition(position);
    cout << "M Finger: " << position.toString() << endl << endl;


    return true;

}*/

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
    bool ret = true;
    ret &= _robotHand->prepare();
    ret &= _explorationFinger->setSynchroProximalAngle(0);

    //cout << "Force: " << _explorationFinger->getContactForce() << endl;
    //Vector cop;
    //_explorationFinger->getContactCoP(cop);
    //cout << "CoP: " << cop.toString() << endl;
}

bool ExploreObject::calibrateHand(){


    _robotHand->calibrate();
    return true;
}

bool ExploreObject::fingerSetAngle(const double angle){
    Vector indexFinger_pos, middleFinger_pos;


    Finger *finger = _robotHand->getIndexFinger();
    finger->getPosition(indexFinger_pos);
    cout << "I: " << indexFinger_pos.toString() << endl;


    finger = _robotHand->getMiddleFinger();

    //finger->setProximalAngle(0);
    //finger->setAngles(0,0, 60);
    // while(!finger->checkMotionDone())
    //    ;
    finger->getPosition(middleFinger_pos);
    cout << "M: " << middleFinger_pos.toString() << endl;


    //finger->setDistalAngle(90,60);
    //while(!finger->checkMotionDone())
    //    ;
    //finger->getPosition(finger_pos);
    //cout << "M: " << finger_pos.toString() << endl;
    //m = finger_pos[2];

    cout << "Dx: " << indexFinger_pos[0] - middleFinger_pos[0] << endl;
    cout << "Dy: " << indexFinger_pos[1] - middleFinger_pos[1] << endl;
    cout << "Dz: " << indexFinger_pos[2] - middleFinger_pos[2] << endl;
    return true;

    /*   bool ret;

    ret = _explorationFinger->setSynchroProximalAngle(angle);
    while(!_explorationFinger->checkMotionDone())
        ;

    _explorationFinger->getPosition(finger_pos);
    cout << "Finger pos: " << finger_pos.toString() << endl;
    _explorationFinger->getAngels(finger_pos);
    cout << "Finger ang: " << finger_pos.toString() << endl;


    return ret;
*/
}

ExploreObject::ExploreObject(yarp::os::ResourceFinder& rf)
{


    _middleFingertipLog.open("middleFingertipLog.csv");
    _indexFingertipLog.open("indexFingertipLog.csv");
    _middleFingerHandLog.open("middleFingerHandLog.csv");
    _indexFingerHandLog.open("indexFingerHandLog.csv");
    _handPoseLog.open("handPoseLog.csv");
    _indexCorrectedLog.open("indexFingerCorrectedLog.csv");

    _dbgtag = "ExploreObject: ";
    // bool failed = false;
    _exploreObjectOnOff = true;
    _exploreObjectValid = true; // Assume it is true, set it to false when something fails
    _stopModule = false;
    _rf = rf;

    //    _maintainContactThread = NULL;
    //_contactSafetyThread = NULL;
    _exploreObjectThread = NULL;
    _exploreObjectGP_thread = NULL;
    _exploreObjectMultifinger_thread = NULL;
    _exploreGPSurface_thread = NULL;

    //// TODO: I save system parameters here that I use in this module.
    /// This is not a good idea. I should change it.
    int readTactilePeriod;
    Bottle& explorationParameters = _rf.findGroup("ExplorationParameters");
    if(!explorationParameters.isNull())
    {
        _explorationThreadPeriod = explorationParameters.check("explorationThreadPeriod", Value(20)).asInt();
        readTactilePeriod = explorationParameters.check("readTactilePeriod", Value(20)).asInt();
    }

    _objectFeaturesThread = new ObjectFeaturesThread(readTactilePeriod,  rf);

}

ExploreObject::~ExploreObject()
{

    _middleFingertipLog.close();
    _indexFingertipLog.close();
    _middleFingerHandLog.close();
    _indexFingerHandLog.close();
    _handPoseLog.close();
    _indexCorrectedLog.close();

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

    if(_exploreObjectMultifinger_thread != NULL)
    {
        if(_exploreObjectMultifinger_thread->isRunning())
            _exploreObjectMultifinger_thread->stop();
        delete(_exploreObjectMultifinger_thread);
        _exploreObjectMultifinger_thread = NULL;
    }

    if(_exploreGPSurface_thread != NULL)
    {
        if(_exploreGPSurface_thread->isRunning())
            _exploreGPSurface_thread->stop();
        delete(_exploreGPSurface_thread);
        _exploreGPSurface_thread = NULL;
    }


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
    return _robotHand->goToStartingPose(_explorationFinger);
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
    _explorationFinger->getPosition(pos);
    _robotHand->setStartingPose(pos, orient);
    return true;
}

bool ExploreObject::setHeight(double height){
    return _robotHand->setHeight(height);
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
    _robotHand->getPose(pos, orient);
    _robotHand->setEndPose(pos, orient);
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

    if(!_exploreObjectValid)
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
        _robotHand->waitMotionDone(0.1, 5);

        // Ge the current position of the arm.
        Vector pos, orient;
        if(!_robotHand->getPose(pos, orient))
        {
            cerr << _dbgtag << "Could not read the arm position, cannot start exploration" << endl;
            return false;
        }


        // Setting the way point to the start of the exploration
        if(!_robotHand->setWayPoint(pos, orient))
        {
            cerr << _dbgtag << "Failed to set the initial waypoint. Aborting exploration!" << endl;
            return false;
        }

        _exploreGPSurface_thread = new ExploreGPSurfaceThread(_explorationThreadPeriod ,
                                                              _robotHand, _explorationFinger, _auxiliaryFinger, "fix", _objectFeaturesThread);



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



bool ExploreObject::startExploringMultifinger(const string& objectName)
{
    bool ret = true;

    cout << "Explore object starting" << endl;

    if(!_exploreObjectValid)
    {
        cerr << _dbgtag << "Cannot start exploring, one or more of the dependencies have not been met" << endl;
        return false;
    }



    if(_exploreObjectOnOff)
    {
        prepHand();
        if(!this->goToStartingPose())
            ret = false;
        _robotHand->waitMotionDone(0.1, 20);
        //_armCartesianController->waitMotionDone(0.1, 20);

        // Ge the current position of the arm.
        Vector pos, orient;
        // pos.resize(3);
        // orient.resize(4);
        if(!_robotHand->getPose(pos, orient))
        {
            cerr << _dbgtag << "Could not read the arm position, cannot start exploration" << endl;
            return false;
        }


        // Setting the way point to the start of the exploration
        if(!_robotHand->setWayPoint(pos, orient))
        {
            cerr << _dbgtag << "Failed to set the initial waypoint. Aborting exploration!" << endl;
            return false;
        }


        _exploreObjectMultifinger_thread =
                new GPExplorationMultifingerThread(_explorationThreadPeriod,
                                                   _robotHand, _explorationFinger, _auxiliaryFinger, objectName, _objectFeaturesThread);


        if(!_exploreObjectMultifinger_thread->start()){
            ret = false;



            cerr << "Exploration could not be started" << endl;
            if(_exploreObjectMultifinger_thread != NULL){
                delete(_exploreObjectMultifinger_thread);
                _exploreObjectMultifinger_thread = NULL;
            }

            return ret;
        }

        _exploreObjectOnOff = false;

        cout << "Exoploring the object using GP\n" << endl;

    }
    else{

        cout << "Warning! Already exploring." << endl;
    }



    return ret;
}


bool ExploreObject::startExploringGP(const string& objectName)
{
    bool ret = true;

    cout << "Explore object starting" << endl;

    if(!_exploreObjectValid)
    {
        cerr << _dbgtag << "Cannot start exploring, one or more of the dependencies have not been met" << endl;
        return false;
    }



    if(_exploreObjectOnOff)
    {
        prepHand();
        if(!this->goToStartingPose())
            ret = false;
        _robotHand->waitMotionDone(0.1, 20);
        //_armCartesianController->waitMotionDone(0.1, 20);

        // Ge the current position of the arm.
        Vector pos, orient;
        // pos.resize(3);
        // orient.resize(4);
        if(!_robotHand->getPose(pos, orient))
        {
            cerr << _dbgtag << "Could not read the arm position, cannot start exploration" << endl;
            return false;
        }


        // Setting the way point to the start of the exploration
        if(!_robotHand->setWayPoint(pos, orient))
        {
            cerr << _dbgtag << "Failed to set the initial waypoint. Aborting exploration!" << endl;
            return false;
        }


        _exploreObjectGP_thread =
                new GPExplorationThread(_explorationThreadPeriod,
                                        _robotHand, _explorationFinger, _auxiliaryFinger, objectName, _objectFeaturesThread);


        if(!_exploreObjectGP_thread->start()){
            ret = false;



            cerr << "Exploration could not be started" << endl;
            if(_exploreObjectGP_thread != NULL){
                delete(_exploreObjectGP_thread);
                _exploreObjectGP_thread = NULL;
            }

            return ret;
        }

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

bool ExploreObject::startExploringGrid(const string objectName)
{
    bool ret = true;

    cout << "Explore object starting" << endl;

    if(!_exploreObjectValid)
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
        //_robotHand->waitMotionDone(0.1, 20);

        // Ge the current position of the arm.
        Vector pos, orient;
        //pos.resize(3);
        //orient.resize(4);
        if(!_robotHand->getPose(pos, orient))
        {
            cerr << _dbgtag << "Could not read the arm position, cannot start exploration" << endl;
            return false;
        }

        // Setting the way point to the start of the exploration
        if(!_robotHand->setWayPoint(pos, orient))
        {
            cerr << _dbgtag << "Failed to set the initial waypoint. Aborting exploration!" << endl;
            return false;
        }




        _exploreObjectThread =
                new GridExplorationThread(_explorationThreadPeriod ,
                                          _robotHand, _explorationFinger, _auxiliaryFinger, objectName, _objectFeaturesThread);

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

bool ExploreObject::startExploring(const std::string &type, const std::string &objectName){

    if(type.compare("gp") == 0){
        startExploringGP(objectName);
    }
    else if(type.compare("grid") == 0){
        startExploringGrid(objectName);
    }
    else if(type.compare("multi") == 0){
        startExploringMultifinger(objectName);
    }
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

        if(_exploreObjectThread != NULL){
            if(_exploreObjectThread->isRunning()){
                _exploreObjectThread->stop();
            }
            delete _exploreObjectThread;
            _exploreObjectThread = NULL;
        }
        if(_exploreObjectGP_thread != NULL){
            if(_exploreObjectGP_thread->isRunning()){
                _exploreObjectGP_thread->stop();
            }
            delete _exploreObjectGP_thread;
            _exploreObjectGP_thread = NULL;
        }
        if(_exploreObjectMultifinger_thread != NULL){
            if(_exploreObjectMultifinger_thread->isRunning()){
                _exploreObjectMultifinger_thread->stop();
            }
            delete _exploreObjectMultifinger_thread;
            _exploreObjectMultifinger_thread = NULL;
        }
        if(_exploreGPSurface_thread != NULL){
            if(_exploreGPSurface_thread->isRunning()){
                _exploreGPSurface_thread->stop();
            }
            delete _exploreGPSurface_thread;
            _exploreGPSurface_thread = NULL;
        }

        cout << "stopped" << endl;

        cout << "Stopped the exploration" << endl;
        _exploreObjectOnOff = true;
    }

    return ret;

}



////////////////// RF module implementation //////////////////


// Attach the port as a server
bool ExploreObject::attach ( yarp::os::Port& source ){
    return this->yarp().attachAsServer(source);
}

bool ExploreObject::configure(yarp::os::ResourceFinder& rf ){

    bool ret = true;

    Bottle robotParameters = rf.findGroup("RobotParameters");
    string robotName = robotParameters.check("robotName", Value("error")).asString();

    try{
        if(robotName.compare("icub") == 0){
            _robotHand = new icubHand(rf);
        }
        else if(robotName.compare("icubSim") == 0){
            _robotHand = new SimHand(rf);
        }
        else{
            return false;
        }
    }catch(std::exception& e){
        cerr << e.what();
        return false;
    }

    //_explorationFinger = _robotHand->getIndexFinger();
    //TODO: use config file to select
    //_explorationFinger = _robotHand->getMiddleFinger();
    //_auxiliaryFinger = _robotHand->getIndexFinger();



    _explorationFinger = _robotHand->getIndexFinger();
    _auxiliaryFinger = _robotHand->getMiddleFinger();

    // Check if exploration finger has force and CoP data
    // We need these two data to be able to explore a surface
    if(!_explorationFinger->hasForceCoP()){
        cerr << _dbgtag << "exploration finger has no force-cop data available to it." << endl;
        return false;
    }

    // Check if in the config file we have a name for the server
    string moduleName = rf.check("moduleName", Value("object-exploration-server"),
                                 "module name (string)").asString().c_str();

    setName(moduleName.c_str());

    _dbgtag = "\n\nexploreObject.cpp: ";






    std::string portName= "/";
    portName+= getName() +"/rpc:i";
    if (!_robotControl_port.open(portName.c_str())) {
        cerr << _dbgtag << ": Unable to open port " << portName << endl;
        return false;
    }


    this->attach(_robotControl_port);



    _objectFeaturesThread->start();

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
    //_robotHand->stopControl();

    /// Store the old context to return the robot to the settings before
    /// this module
    //_armCartesianController->restoreContext(_cartCtrlStartupIDstartupID);
    //_deviceController.close();


    //_deviceController_joint.close();

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



