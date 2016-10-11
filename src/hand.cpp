#include "hand.h"
#include <iostream>
#include <yarp/os/Value.h>
#include <stdexcept>
#include <deque>

namespace objectExploration {
using std::cerr;
using std::endl;
using std::cout;

using yarp::os::Value;

/*
 * 192 for hand data, where 1-60 are taxels of fingertips (12 each in this order:
 * index, middle, ring, little, thumb); 61-96 zeros; 97-144 palm taxels
 * (inside these, 108, 120, 132, and 140 are thermal pads ~ 0s); 145-192 zeros.
 */

Hand::Hand(ResourceFinder rf){

    _desiredStartingPosition.resize(3);
    _desiredStartingOrientation.resize(4);
    _desiredStartingPose_isValid = false;
    _desiredEndPosition.resize(3);
    _desiredEndOrientation.resize(4);
    _desiredEndPose_isValid = false;

    _armEncoders = NULL;
    _armJointModeCtrl = NULL;
    _armJointPositionCtrl = NULL;
    _armCartesianCtrl = NULL;

    _indexFinger = NULL;
    _middleFinger = NULL;
    _ringFinger = NULL;
    _littleFinger = NULL;
    _thumb = NULL;

    configure(rf);

}

bool Hand::multiContact(double angle){


   _middleFinger->setAngles(0, angle, 30);
}

bool Hand::setWayPoint(yarp::sig::Vector pos){
    return setWayPoint(pos, _desiredStartingOrientation);
}

bool Hand::prepare(){

    bool ret = true;

    setAbduction(0);
    _indexFinger->prepare();
    _middleFinger->prepare();
    _ringFinger->prepare();
    _littleFinger->prepare();
    _thumb->prepare();

    return true;


}

bool Hand::calibrate(){

   // double min, max;
   // for (int iJoint = 0; iJoint < 16; iJoint++){
   // _armControlLimits->getLimits(iJoint, &min, &max );
   // cout << "J " << iJoint << " min: " << min << ", max: " << max << endl;
   // }
   // return true;
    open();
    _indexFinger->calibrate();
    open();
    _middleFinger->calibrate();


    Vector position;
    position.clear();
    _indexFinger->getPosition(position);
    cout << "I: " << position.toString() << endl;
    position.clear();
    _middleFinger->getPosition(position);
    cout << "M: " << position.toString() << endl;

//    cout << "Little" << endl;
//    _littleFinger->getPosition(position);
//    cout << position.toString() << endl;

    //double min, max;

    //_armControlLimits->getLimits(11, &min, &max);
    //std::cout << "Min: " << min << "Max: " << max << std::endl;

    //std::deque<yarp::dev::IControlLimits*> calib;
   // calib.push_back(_armControlLimits);
    //_indexFinger->alignJointsBounds(calib);

    //calib.push_back(_armControlLimits);

/*    _indexFinger->printJointLimits();
    _indexFinger->calibrate2();
    _indexFinger->printJointLimits();
*/

/*    _indexFinger->open();
    _thumb->open();
    _middleFinger->open();

    _indexFinger->calibrate();*/

    return true;
}

bool Hand::checkOpenMotionDone(){

    bool ret_index, ret_middle, ret_ring, ret_little, ret_thumb;
    ret_index = _indexFinger->checkMotionDone();
    ret_middle = _middleFinger->checkMotionDone();

    ret_thumb = _thumb->checkMotionDone();

    return(ret_index && ret_middle && ret_thumb);

}

bool Hand::open(){

    setAbduction(0);

    // Ask all fingers to open
    _indexFinger->open();
    _middleFinger->open();
    _ringFinger->open();
    //_littleFinger->open(); //This not needed because it is linked to the ring finger
    _thumb->open();
}

bool Hand::checkMotionDone(bool *motionDone){
    return _armCartesianCtrl->checkMotionDone(motionDone);
}

bool Hand::setAbduction(double angle, double speed){

    Bottle& msg = _abuductionPort_out.prepare();
    msg.clear();
    msg.addDouble(angle);
    _abuductionPort_out.writeStrict();

    _armJointPositionCtrl->setRefSpeed(ABDUCTION, speed);
    if(!_armJointPositionCtrl->positionMove(ABDUCTION, angle))
    {
        cerr << _dbgtag << "Falied to move to the requsted positions." << endl;
    }
}

void Hand::updateSafeWorkspace()
{
    if(_desiredStartingPose_isValid) // && _desiredEndPose_isValid)
    {
        _safeWorkspace.minX = _desiredStartingPosition[0] - 0.30; //Maximum width 13 cm + 2 cm leeway
        _safeWorkspace.maxX = _desiredStartingPosition[0] + 0.10;
        _safeWorkspace.minZ = _desiredStartingPosition[2] - 0.04; //TODO: set it to fixed value in the config file
        _safeWorkspace.maxZ = _desiredStartingPosition[2] + 0.15; //TODO: set them to fixed values in the config files

    }

    if(_desiredStartingPose_isValid && _desiredEndPose_isValid)
    {
        if(_desiredStartingPosition[1] < _desiredEndPosition[1])
        {
            _safeWorkspace.minY = _desiredStartingPosition[1] - 0.08;
            _safeWorkspace.maxY = _desiredEndPosition[1] + 0.08;

        }
        else
        {
            _safeWorkspace.minY = _desiredEndPosition[1] - 0.08;
            _safeWorkspace.maxY = _desiredStartingPosition[1] + 0.08;

        }
    }
}

void Hand::setStartingPose ( Vector& pos, Vector& orient )
{
    if(pos[0] > _safeWorkspace.disasterX)
    {
        cerr << _dbgtag << "Invalid x position" << endl;
        return;
    }
    //cout << "Starting pose set" << endl;
    _desiredStartingPosition = pos;
    _desiredStartingOrientation = orient;
    _desiredStartingPose_isValid = true;
    //updateSafeWorkspace(pos, orient);

    updateSafeWorkspace(); // This must be done after isValid is set to true;
    printPose(pos, orient);

}

void Hand::setEndPose ( Vector& pos, Vector& orient )
{
    if(pos[0] > _safeWorkspace.disasterX)
    {
        cerr << _dbgtag << "Invalid x position" << endl;
        return;
    }
    _desiredEndPosition = pos;
    _desiredEndOrientation = orient;
    _desiredEndPose_isValid = true;
    updateSafeWorkspace();
    printPose(pos, orient);
}

bool Hand::getPose(Vector &pos, Vector &orient)
{


    pos.resize(3);
    orient.resize(4);
    return _armCartesianCtrl->getPose(pos, orient);

}

bool Hand::goToPoseSync(yarp::sig::Vector& pos, yarp::sig::Vector& orient, double timeout){

    bool ret;
    ret =  _armCartesianCtrl->goToPoseSync(pos, orient);
    if(timeout > 0){
        _armCartesianCtrl->waitMotionDone(0.1, timeout);
    }
    return ret;


}

bool Hand::getStartingPose ( Vector& pos, Vector& orient )
{
    if(_desiredStartingPose_isValid)
    {
        pos = _desiredStartingPosition;
        orient = _desiredStartingOrientation;
    }
    return _desiredStartingPose_isValid;

}

bool Hand::setHeight(double height){

    if(height < _safeWorkspace.minZ){
        cerr << _dbgtag << "Exceeded the min z-axis limit ( " << _safeWorkspace.minZ << " ): " << height << endl;
        height = _safeWorkspace.minZ;
    }

    if(height > _safeWorkspace.maxZ){
        cerr << _dbgtag << "Exceeded the max z-axis limit ( " << _safeWorkspace.maxZ << " ): " << height << endl;
        height = _safeWorkspace.maxZ;
    }
    _desiredStartingPosition[2] = height;
    _desiredEndPosition[2] = height;

    return true;
}

bool Hand::goToStartingPose(Finger * explorationFinger){

    Vector desiredFingerPos;
    Vector desiredArmPos, desiredArmOrient;
    Vector currentArmPos, currentArmOrient;


    if(getStartingPose(desiredFingerPos, desiredArmOrient))
    {

        // Move the arm up before moving sideways
        explorationFinger->toArmPosition(desiredFingerPos, desiredArmPos);

        // Get the current arm pose
        getPose(currentArmPos, currentArmOrient);
        currentArmPos[2] = desiredArmPos[2];
        goToPoseSync(currentArmPos, desiredArmOrient,10);

        bool motionDone = false;
        while(!motionDone){
            checkMotionDone(&motionDone);
        }
        goToPoseSync(desiredArmPos, desiredArmOrient, 10);

        return true;
    }

    return false;
}

bool Hand::getEndPose(yarp::sig::Vector &pos, yarp::sig::Vector &orient){
    if(_desiredEndPose_isValid)
    {
        pos = _desiredEndPosition;
        orient = _desiredEndOrientation;
    }
    return _desiredEndPose_isValid;
}

bool Hand::goToEndPose(){
    Vector desiredFingerPos;
    Vector desiredArmPos, desiredArmOrient;
    Vector currentArmPos, currentArmOrient;

    if(getEndPose(desiredFingerPos, desiredArmOrient))
    {
        _indexFinger->toArmPosition(desiredFingerPos, desiredArmPos);

        // Move the hand up before moving sidways.
        getPose(currentArmPos, currentArmOrient);
        currentArmPos[2] = desiredArmPos[2];
        goToPoseSync(currentArmPos, currentArmOrient, 10);
        goToPoseSync(desiredArmPos, desiredArmOrient, 10);



    }
    return false;
}

void Hand::stopControl(){
    _armCartesianCtrl->stopControl();
}

bool Hand::setWayPoint( Vector pos, Vector orient ){
    _wayPoint_isValid = true;

    // Breaching this will be disasterous for the robot!
    if(pos[0] >= _safeWorkspace.disasterX)
    {
        cerr << _dbgtag << "Cannot have positive x-axis value" << endl;
        _wayPoint_isValid = false;
        return false;
    }

    if(pos[0] < _safeWorkspace.minX )
    {
        cerr << _dbgtag << "X position outside allowable range ( "
             << _safeWorkspace.minX << ", "
             <<  _safeWorkspace.maxX << "): " << pos[0] << endl;

        pos[0] = _safeWorkspace.minX;
        _wayPoint_isValid =  false;
        //return _wayPoint_isValid;
    }

    if( pos[0] > _safeWorkspace.maxX)
    {
        cerr << _dbgtag << "X position outside allowable range ( "
             << _safeWorkspace.minX << ", "
             <<  _safeWorkspace.maxX << "): " << pos[0] << endl;

        pos[0]  = _safeWorkspace.maxX;
        _wayPoint_isValid = false;
        //return _wayPoint_isValid;
    }


    if(pos[1] < _safeWorkspace.minY )
    {
        cerr << _dbgtag << "Y position outside allowable range ( "
             << _safeWorkspace.minY << ", "
             <<  _safeWorkspace.maxY << "): " << pos[1] << endl;

        pos[1] = _safeWorkspace.minY;
        _wayPoint_isValid = false;

    }

    if( pos[1] > _safeWorkspace.maxY)
    {
        cerr << _dbgtag << "Y position outside allowable range ( "
             << _safeWorkspace.minY << ", "
             <<  _safeWorkspace.maxY << "): " << pos[1] << endl;

        pos[1] = _safeWorkspace.maxY;
        _wayPoint_isValid = false;
        //return _wayPoint_isValid;
    }

    if(pos[2] < _safeWorkspace.minZ)
    {

        cerr << _dbgtag << "Exceeded the min z-axis limit ( " << _safeWorkspace.minZ << " ): " << pos[2] << endl;
        pos[2] = _safeWorkspace.minZ;

        if(_wayPointPos[2] == _safeWorkspace.minZ)
            _wayPoint_isValid = false;
    }

    if(pos[2] > _safeWorkspace.maxZ)
    {
        cerr << _dbgtag << "Exceeded the max z-axis limit ( " << _safeWorkspace.maxZ << " ): " << pos[2] << endl;
        pos[2] = _safeWorkspace.maxZ;

        if(_wayPointPos[2] == _safeWorkspace.maxZ)
            _wayPoint_isValid = false;
    }


    _wayPointPos = pos;
    _wayPointOrient = orient;

    return _wayPoint_isValid;

}

bool Hand::setWayPointGP(yarp::sig::Vector pos, yarp::sig::Vector orient)
{
    // Breaching this will be disasterous for the robot!
    if(pos[0] >= _safeWorkspace.disasterX)
    {
        cerr << _dbgtag << "Cannot have positive x-axis value" << endl;
        _wayPoint_isValid = false;
        return false;
    }


    pos[2] = _desiredStartingPosition[2];
    setWayPoint (pos, orient );
    _wayPoint_isValid = true;

    return true;
}

bool Hand::getWayPoint ( Vector& pos, Vector& orient, bool invalidateWayPoint )
{

    bool ret = _wayPoint_isValid;
    //if(_wayPoint_isValid)
    //{
    pos = _wayPointPos;
    orient = _wayPointOrient;
    _wayPoint_isValid = !invalidateWayPoint;
    //  return true;
    //}
    return ret;
}

void Hand::printPose ( Vector& pos, Vector& orient )
{
    cout << "Position: " << pos.toString() << endl;
    cout << "Orientation: " << orient.toString() << endl;
}

void Hand::configure(yarp::os::ResourceFinder rf){



    ///// Set a safe workspace for the robot
    //// This get updated when I read the
    /// Starting and ending positions

    _safeWorkspace.minX = -0.4;
    _safeWorkspace.maxX = -0.2;
    _safeWorkspace.minY =  0; // This works for both arms
    _safeWorkspace.maxY =  0; // This works for both arms
    _safeWorkspace.minZ = -0.1;
    _safeWorkspace.maxZ =  0.1;
    _safeWorkspace.disasterX = -0.2; // Beyond this point you will break the hand


    _moduleName = rf.check("moduleName", Value("default-module"),
                           "module name (string)").asString().c_str();


    Bottle &robotParameters = rf.findGroup("RobotParameters");
    if(robotParameters.isNull()){
        return;
    }

    // Read the arm configuration
    _whichHand = robotParameters.check("arm", Value("Error")).asString();
    _robotName = robotParameters.check("robotName", Value("icubSim")).asString();
    string controller = robotParameters.check("controller", Value("Error")).asString();
    string controllerName = robotParameters.check("controllerName", Value("Error")).asString();
    int trajectoryTime = robotParameters.check("trajectoryTime", Value(5)).asInt();

    //string whichFinger = robotParameters.check("whichFinger", Value("index")).asString();
    //string moduleName = "object-exploration" ;

    cout << "Read the following configuration from the file:" << endl;
    cout << "Robot name: " << _robotName << endl;
    cout << "Arm: " << _whichHand << endl;
    cout << "Controller: " << controller << endl;
    cout << "Controller name: " << controllerName << endl;
    cout << "Trajectory time: " << trajectoryTime << endl;



    ///////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////// Configure the Cartersian controller ////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////

    yarp::os::Property deviceOptions;
    deviceOptions.put("device", controller);
    deviceOptions.put("local", "/" + _moduleName + "/" + controllerName + "/" + _whichHand + "_arm");
    deviceOptions.put("remote", "/" + _robotName + "/" + controllerName + "/" + _whichHand + "_arm");

    //cout << "Device options: " << deviceOptions.toString() << endl;

    if(!_deviceController.open(deviceOptions))
    {
        cerr << _dbgtag << "Failed to open the device: " << controller << endl;

       throw std::runtime_error("Failed to open the device: " + controller );
    }

    // Open a Cartesian controller

    if(!_deviceController.view(_armCartesianCtrl)){
        cerr << _dbgtag << "Failed to get a Cartesian view" << endl;
        throw std::runtime_error("Failed to get a Cartesian view");
    }

    // Remember the contorller context ID, restore when closing the port
    _armCartesianCtrl->storeContext(&_cartCtrlStartupIDstartupID);

    // Set the trajectory time
    //cout << "Trajectory time: " << trajectoryTime << endl;
    _armCartesianCtrl->setTrajTime(trajectoryTime);
    //_armCartesianCtrl->setInTargetTol(2.5/1000); // half of
    //strictTolerence();

    // Enable the torso movement

    Vector curDof;
    _armCartesianCtrl->getDOF(curDof);
    //cout<<"["<<curDof.toString()<<"]"<<endl;  // [0 0 0 1 1 1 1 1 1 1] will be printed out
    Vector newDof(3);
    newDof[0]=1;    // torso pitch: 1 => enable
    newDof[1]=2;    // torso roll:  2 => skip
    newDof[2]=1;    // torso yaw:   1 => enable
    _armCartesianCtrl->setDOF(newDof,curDof);
    //cout<<"["<<curDof.toString()<<"]"<<endl;  // [1 0 1 1 1 1 1 1 1 1] will be printed out

    //_armCartesianCtrl->setPosePriority("orientation");



    //////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////        Joint Control                     /////////////////
    //////////////////////////////////////////////////////////////////////////////////////

    yarp::os::Property optionsJnt;
    optionsJnt.put("device", "remote_controlboard");
    optionsJnt.put("local", "/" + _moduleName + "/" + _whichHand + "_arm/joint");                 //local port names
    optionsJnt.put("remote", "/" + _robotName + "/" + _whichHand + "_arm");



    //cout << "Device options: " << optionsJnt.toString() << endl;

    if(!_deviceController_joint.open(optionsJnt))
    {
        cerr << _dbgtag << "Failed to open the device: " << "urgh" << endl; //systemParameters.getControllerType() << endl;

        throw std::runtime_error("Failed to open the device: remote_controlboard");

    }

    //open an armcontroller_mode view
    if(!_deviceController_joint.view(_armJointModeCtrl))
    {
        cerr << _dbgtag << "Failed to open arm joint mode controller" << endl;
         throw std::runtime_error("Failed to open arm joint mode controller");

    }

    // Open arm limits view
    if(!_deviceController_joint.view(_armControlLimits)){
        cerr << _dbgtag << "Failed to open Control Limists view" << endl;
        throw std::runtime_error("Failed to open arm encoder view");
    }

    // Open an encoder view
    if(!_deviceController_joint.view(_armEncoders))
    {
        cerr << _dbgtag << "Failed to open Encoder view" << endl;
        throw std::runtime_error("Failed to open arm encoder view");
    }

    if(!_deviceController_joint.view(_armJointPositionCtrl))
    {
        cerr << _dbgtag << "Failed to open joint position controller view" << endl;
        throw std::runtime_error("Failed to open joint position controller view");
        //_exploreObjectValid = false;
        //return false;
    }

    int armJointsNum;
    _armJointPositionCtrl->getAxes(&armJointsNum);
    // Set reference speeds
    std::vector<double> refSpeeds(armJointsNum, 0);
    _armJointPositionCtrl->getRefSpeeds(&refSpeeds[0]);
    for (int i = 11; i < 15; ++i) {
        refSpeeds[i] = 50;
    }
    _armJointPositionCtrl->setRefSpeeds(&refSpeeds[0]);

    //_armCartesianController->getPose()



    //////////////////////////// Starting and End pose  ///////////////////

    Bottle& explorationParameters = rf.findGroup("ExplorationParameters");
    Bottle* startingPose;
    Bottle* endPose;
    if(!explorationParameters.isNull())
    {
        startingPose = explorationParameters.find("startingPose").asList();
        endPose = explorationParameters.find("endPose").asList();

    }

    if(startingPose->size() < 7)
        cout << "startingPose is invalid" << endl;
    else
    {
        Vector pos, orient;
        pos.resize(3);
        orient.resize(4);

        for(int i = 0; i < 3; i++)
            pos[i] = startingPose->get(i).asDouble();
        for(int i = 3; i < 7; i++)
            orient[i-3] = startingPose->get(i).asDouble();

        setStartingPose(pos, orient);


    }
    if(endPose->size() < 7)
        cout << "End pose is invalid!" << endl;
    else
    {

        Vector pos, orient;
        pos.resize(3);
        orient.resize(4);

        for(int i = 0; i < 3; i++)
            pos[i] = endPose->get(i).asDouble();
        for(int i = 3; i < 7; i++)
            orient[i-3] = endPose->get(i).asDouble();

        setEndPose(pos, orient);

    }


    if(_desiredStartingPose_isValid)
    {

        cout << "Starting position: " << _desiredStartingPosition.toString() << endl;
        cout << "Starting orientation: " << _desiredStartingOrientation.toString() << endl;
    }

    if(_desiredEndPose_isValid)
    {
        cout << "End position: " << _desiredEndPosition.toString() << endl;
        cout << "End orientation: " << _desiredEndOrientation.toString() << endl;
    }


    _abuductionPort_out.open("/object-exploration/" + _whichHand + "_hand/abduction/control:o");



    // ToDo: clean it up
    _rawTactileData_in.open("/object-exploration/rawTactile:i");
    yarp::os::Network::connect("/icub/skin/left_hand", "/object-exploration/rawTactile:i");
}

Hand::~Hand(){
    _abuductionPort_out.close();
}

void Hand::waitMotionDone(const double period, const double timeout){
    _armCartesianCtrl->waitMotionDone(period, timeout);
}

/*void Hand::relaxTolerence(){
    //_armCartesianCtrl->setInTargetTol(20/100);
}

void Hand::strictTolerence(){
    //_armCartesianCtrl->setInTargetTol(2.5/100);
}

*/

bool SimHand::configure(yarp::os::ResourceFinder& rf){
    // Common configuration
    //Hand::configure(rf);




    t_controllerData ctrlData;
    ctrlData.whichHand = _whichHand;
    ctrlData.armJointModeCtrl = _armJointModeCtrl;
    ctrlData.armEncoder = _armEncoders;
    ctrlData.armJointPositionCtrl = _armJointPositionCtrl;

    ctrlData.armCartesianCtrl = _armCartesianCtrl;

    FingerFactory fingerCreator;

    _thumb = fingerCreator.createFinger("thumb", "icubSim", ctrlData);
    _indexFinger = fingerCreator.createFinger("index", "icubSim", ctrlData);
    _middleFinger = fingerCreator.createFinger("middle", "icubSim", ctrlData);
}

icubHand::icubHand(yarp::os::ResourceFinder &rf):Hand(rf){
    configure(rf);
}

SimHand::SimHand(yarp::os::ResourceFinder &rf):Hand(rf){
    configure(rf);
}

bool icubHand::configure(yarp::os::ResourceFinder rf){

    // Common configuration
    //Hand::configure(rf);

    string localPort = "/" + _moduleName + "/" + _whichHand + "_hand/analog:i";
    string remotePort = "/" + _robotName + "/" + _whichHand + "_hand/analog:o";

    if(!_fingerEncoders.open(localPort) || ! yarp::os::Network::connect(remotePort, localPort)){
       std::runtime_error("Could not connect to: " + remotePort);
    }


    t_controllerData ctrlData;
    ctrlData.whichHand = _whichHand;
    ctrlData.armJointModeCtrl = _armJointModeCtrl;
    ctrlData.armEncoder = _armEncoders;
    ctrlData.armJointPositionCtrl = _armJointPositionCtrl;
    ctrlData.fingerEncoders = &_fingerEncoders;
    ctrlData.armCartesianCtrl = _armCartesianCtrl;
    ctrlData.rawTactileData_in = &_rawTactileData_in;
    ctrlData.armControlLimits = _armControlLimits;

    FingerFactory fingerCreator;

    _thumb = fingerCreator.createFinger("thumb", "icub", ctrlData);
    _indexFinger = fingerCreator.createFinger("index", "icub", ctrlData);
    _middleFinger = fingerCreator.createFinger("middle", "icub", ctrlData);
    _ringFinger = fingerCreator.createFinger("ring", "icub", ctrlData);
    _littleFinger = fingerCreator.createFinger("little", "icub", ctrlData);

    return true;

}



}
