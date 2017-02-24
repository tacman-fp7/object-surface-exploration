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

/**
 * @brief ExploreObject::setSafetyThreshold
 * @param threshold
 * @return
 * Sets the force threshold used by contact safety thread, which stops the
 * cartesian controller when this threshold is exceeded.
 */
bool ExploreObject::setSafetyThreshold(const double threshold){
    _robotHand->setContactSafetyForceThrehold(threshold);
    _exploreObject_thread->setContactSafetyForceThreshold(threshold);

    return true;
}

/**
 * @brief ExploreObject::alignFingers
 * @return
 * Get position data for the index finger and the middle finger
 * to be used for offline calculation of the alignment
 * matric using ICP
 */
bool ExploreObject::alignFingers(){
#define MAX_POSITIONS 200

    Finger *indexFinger = _robotHand->getIndexFinger();
    Finger *middleFinger = _robotHand->getMiddleFinger();

    std::ofstream _middleFingertipLog;
    std::ofstream _indexFingertipLog;
    std::ofstream _indexFingerHandLog;
    std::ofstream _middleFingerHandLog;
    std::ofstream _handPoseLog;
    std::ofstream _indexCorrectedLog;


    // Open the files to store the positions
    _middleFingertipLog.open("middleFingertipLog.csv");
    _indexFingertipLog.open("indexFingertipLog.csv");
    _middleFingerHandLog.open("middleFingerHandLog.csv");
    _indexFingerHandLog.open("indexFingerHandLog.csv");
    _handPoseLog.open("handPoseLog.csv");
    _indexCorrectedLog.open("indexFingerCorrectedLog.csv");


    yarp::sig::Vector position;
    for (int i = 0; i < MAX_POSITIONS; i++ ){
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

    // Close the files
    _middleFingertipLog.close();
    _indexFingertipLog.close();
    _middleFingerHandLog.close();
    _indexFingerHandLog.close();
    _handPoseLog.close();
    _indexCorrectedLog.close();
    return true;

}




bool ExploreObject::validatePositionsEnable(){
    //_exploreObject_thread->enableValidatePositions();
    return true;
}

bool ExploreObject::validatePositionsDisable(){
    //_exploreObject_thread->disaleValidatePositions();
    return true;
}

bool ExploreObject::nRepeatsSet(const int32_t nRepeats){
    _exploreObject_thread->setNRepeats(nRepeats);
    return true;
}

bool ExploreObject::refineModelEnable(){
    //_exploreObject_thread->enableRefiningModel();
    return true;
}

bool ExploreObject::refineModelDisable(){
    //_exploreObject_thread->disableRefiningModel();
    return true;
}

bool ExploreObject::openHand(){
    return _robotHand->open();
}

bool ExploreObject::prepHand(){
    bool ret = true;
    ret &= _robotHand->prepare();
    ret &= _explorationFinger->setSynchroProximalAngle(0);

}

bool ExploreObject::calibrateHand(){

    Vector encoders;

    _robotHand->getIndexFinger()->readEncoders(encoders);
    cout << "Index Finger: " << encoders.toString() << endl;

    _robotHand->getMiddleFinger()->readEncoders(encoders);
    cout << "Middle Finger: " << encoders.toString() << endl;
    //_robotHand->calibrate();
    return true;
}

bool ExploreObject::fingerSetAngle(const double angle){
    Vector indexFinger_pos, middleFinger_pos;


    Finger *finger = _robotHand->getIndexFinger();
    finger->getPosition(indexFinger_pos);
    cout << "I: " << indexFinger_pos.toString() << endl;


    finger = _robotHand->getMiddleFinger();


    finger->getPosition(middleFinger_pos);
    cout << "M: " << middleFinger_pos.toString() << endl;




    cout << "Dx: " << indexFinger_pos[0] - middleFinger_pos[0] << endl;
    cout << "Dy: " << indexFinger_pos[1] - middleFinger_pos[1] << endl;
    cout << "Dz: " << indexFinger_pos[2] - middleFinger_pos[2] << endl;
    return true;

}

ExploreObject::ExploreObject(yarp::os::ResourceFinder& rf)
{





    _dbgtag = "ExploreObject: ";
    // bool failed = false;
    _exploreObjectOnOff = true;
    _exploreObjectValid = true; // Assume it is true, set it to false when something fails
    _stopModule = false;
    _rf = rf;

    //    _maintainContactThread = NULL;
    //_contactSafetyThread = NULL;
    _exploreObject_thread = NULL;
   // _exploreObject_thread = NULL;
    //_exploreObject_thread = NULL;
    //_exploreObject_thread = NULL;

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

    if(_exploreObject_thread != NULL)
    {
        if(_exploreObject_thread->isRunning())
            _exploreObject_thread->stop();

        delete(_exploreObject_thread);
        _exploreObject_thread = NULL;
    }

 /*   if(_exploreObject_thread != NULL)
    {
        if(_exploreObject_thread->isRunning())
            _exploreObject_thread->stop();
        delete(_exploreObject_thread);
        _exploreObject_thread = NULL;
    }

    if(_exploreObject_thread != NULL)
    {
        if(_exploreObject_thread->isRunning())
            _exploreObject_thread->stop();
        delete(_exploreObject_thread);
        _exploreObject_thread = NULL;
    }

    if(_exploreObject_thread != NULL)
    {
        if(_exploreObject_thread->isRunning())
            _exploreObject_thread->stop();
        delete(_exploreObject_thread);
        _exploreObject_thread = NULL;
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

        _exploreObject_thread = new ExploreGPSurfaceThread(_explorationThreadPeriod ,
                                                              _robotHand, _explorationFinger, _auxiliaryFinger, "fix", _objectFeaturesThread);



        if(!_exploreObject_thread->start())
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

    cout << "Explore object using multi finger GP" << endl;

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


        _exploreObject_thread =
                new GPExplorationMultifingerThread(_explorationThreadPeriod,
                                                   _robotHand, _explorationFinger, _auxiliaryFinger, objectName, _objectFeaturesThread);


        if(!_exploreObject_thread->start()){
            ret = false;



            cerr << _dbgtag << "Exploration could not be started" << endl;
            if(_exploreObject_thread != NULL){
                delete(_exploreObject_thread);
                _exploreObject_thread = NULL;
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

    cout << "Explore object using single finger GP" << endl;

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


        _exploreObject_thread =
                new GPExplorationThread(_explorationThreadPeriod,
                                        _robotHand, _explorationFinger, _auxiliaryFinger, objectName, _objectFeaturesThread);


        if(!_exploreObject_thread->start()){
            ret = false;



            cerr << "Exploration could not be started" << endl;
            if(_exploreObject_thread != NULL){
                delete(_exploreObject_thread);
                _exploreObject_thread = NULL;
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
    if(_exploreObject_thread != NULL)
    {
        //_exploreObject_thread->enableSurfaceSampling();
    }

    return true;
}

bool ExploreObject::disableSurfaceSampling()
{

    if(_exploreObject_thread != NULL)
    {
        //_exploreObject_thread->disableSurfaceSampling();
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

        prepHand();
        if(!this->goToStartingPose())
            ret = false;


        // Ge the current position of the arm.
        Vector pos, orient;;
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




        _exploreObject_thread =
                new GridExplorationThread(_explorationThreadPeriod ,
                                          _robotHand, _explorationFinger, _auxiliaryFinger, objectName, _objectFeaturesThread);

        if(!_exploreObject_thread->start())
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
        cout << "Warning! Nothing to stop, are you sure?" << endl;
    }
    else
    {



        // 0 is invalid state!
        _objectFeaturesThread->updateContactState(0);

        if(_exploreObject_thread != NULL){
            if(_exploreObject_thread->isRunning()){
                _exploreObject_thread->stop();
            }
            delete _exploreObject_thread;
            _exploreObject_thread = NULL;
        }
       /* if(_exploreObject_thread != NULL){
            if(_exploreObject_thread->isRunning()){
                _exploreObject_thread->stop();
            }
            delete _exploreObject_thread;
            _exploreObject_thread = NULL;
        }
        if(_exploreObject_thread != NULL){
            if(_exploreObject_thread->isRunning()){
                _exploreObject_thread->stop();
            }
            delete _exploreObject_thread;
            _exploreObject_thread = NULL;
        }
        if(_exploreObject_thread != NULL){
            if(_exploreObject_thread->isRunning()){
                _exploreObject_thread->stop();
            }
            delete _exploreObject_thread;
            _exploreObject_thread = NULL;
        }
        */

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



