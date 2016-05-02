#include "hand.h"
#include <iostream>
#include <yarp/os/Value.h>

namespace objectExploration {
using std::cerr;
using std::endl;
using std::cout;

using yarp::os::Value;

Hand::Hand(ResourceFinder rf){

    _desiredStartingPosition.resize(3);
    _desiredStartingOrientation.resize(4);
    _desiredStartingPose_isValid = false;
    _desiredEndPosition.resize(3);
    _desiredEndOrientation.resize(4);
    _desiredEndPose_isValid = false;
    configure(rf);

}

bool Hand::prepare(){

    bool ret = true;

    // Prepare all fingers
    ret = ret && setAbduction(20);
    ret = ret && _indexFinger->prepare();
    ret = ret && _thumb->prepare();

    return true;


}

bool Hand::calibrate(){

    _indexFinger->calibrate();

    return true;
}

bool Hand::checkOpenMotionDone(){

    bool ret_index, ret_thumb;
    ret_index = _indexFinger->checkMotionDone();
    ret_thumb = _thumb->checkMotionDone();

    return(ret_index && ret_thumb);

}

bool Hand::open(){

    setAbduction(20);

    // Ask all fingers to open
    _indexFinger->open();
    _thumb->open();
}

bool Hand::checkMotionDone(bool *motionDone){
    return _armCartesianCtrl->checkMotionDone(motionDone);
}

bool Hand::setAbduction(double angle, double speed){

    _armJointPositionCtrl->setRefSpeed(ABDUCTION, speed);
    if(!_armJointPositionCtrl->positionMove(ABDUCTION, angle))
    {
        cerr << _dbgtag << "Falied to move to the requsted positions." << endl;
    }
}

void Hand::updateRobotReachableSpace()
{
    if(_desiredStartingPose_isValid) // && _desiredEndPose_isValid)
    {
        _robotReachableSpace.minX = _desiredStartingPosition[0] - 0.30; //Maximum width 13 cm + 2 cm leeway
        _robotReachableSpace.maxX = _desiredStartingPosition[0] + 0.10;
        _robotReachableSpace.minZ = _desiredStartingPosition[2] - 0.05;
        _robotReachableSpace.maxZ = _desiredStartingPosition[2] + 0.05;

    }

    if(_desiredStartingPose_isValid && _desiredEndPose_isValid)
    {
        if(_desiredStartingPosition[1] < _desiredEndPosition[1])
        {
            _robotReachableSpace.minY = _desiredStartingPosition[1] - 0.08;
            _robotReachableSpace.maxY = _desiredEndPosition[1] + 0.08;

        }
        else
        {
            _robotReachableSpace.minY = _desiredEndPosition[1] - 0.08;
            _robotReachableSpace.maxY = _desiredStartingPosition[1] + 0.08;

        }
    }
}

void Hand::setStartingPose ( Vector& pos, Vector& orient )
{
    if(pos[0] > _robotReachableSpace.disasterX)
    {
        cerr << _dbgtag << "Invalid x position" << endl;
        return;
    }
    //cout << "Starting pose set" << endl;
    _desiredStartingPosition = pos;
    _desiredStartingOrientation = orient;
    _desiredStartingPose_isValid = true;
    updateRobotReachableSpace(); // This must be done after isValid is set to true;
    printPose(pos, orient);

}

void Hand::setEndPose ( Vector& pos, Vector& orient )
{
    if(pos[0] > _robotReachableSpace.disasterX)
    {
        cerr << _dbgtag << "Invalid x position" << endl;
        return;
    }
    _desiredEndPosition = pos;
    _desiredEndOrientation = orient;
    _desiredEndPose_isValid = true;
    updateRobotReachableSpace();
    printPose(pos, orient);
}

bool Hand::getPose(Vector &pos, Vector &orient)
{


    pos.resize(3);
    orient.resize(4);
    return _armCartesianCtrl->getPose(pos, orient);

}

bool Hand::goToPoseSync(yarp::sig::Vector pos, yarp::sig::Vector orient, double timeout){

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

bool Hand::goToStartingPose(){

    Vector desiredFingerPos;
    Vector desiredArmPos, desiredArmOrient;
    Vector currentArmPos, currentArmOrient;


    if(getStartingPose(desiredFingerPos, desiredArmOrient))
    {

      // Move the arm up before moving sideways
        _indexFinger->toArmPosition(desiredFingerPos, desiredArmPos);

        // Get the current arm pose
       getPose(currentArmPos, currentArmOrient);
        currentArmPos[2] = desiredArmPos[2];
        goToPoseSync(currentArmPos, currentArmOrient,10);

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
        return goToPoseSync(currentArmPos, currentArmOrient, 10);


    }
    return false;
}

void Hand::stopControl(){
    _armCartesianCtrl->stopControl();
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

    _robotReachableSpace.minX = -0.4;
    _robotReachableSpace.maxX = -0.2;
    _robotReachableSpace.minY =  0; // This works for both arms
    _robotReachableSpace.maxY =  0; // This works for both arms
    _robotReachableSpace.minZ = -0.1;
    _robotReachableSpace.maxZ =  0.1;
    _robotReachableSpace.disasterX = -0.2; // Beyond this point you will break the hand


    string moduleName = rf.check("moduleName", Value("object-exploration-server"),
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

    string whichFinger = robotParameters.check("whichFinger", Value("index")).asString();
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
    deviceOptions.put("local", "/" + moduleName + "/" + controllerName + "/" + _whichHand + "_arm");
    deviceOptions.put("remote", "/" + _robotName + "/" + controllerName + "/" + _whichHand + "_arm");

    //cout << "Device options: " << deviceOptions.toString() << endl;

    if(!_deviceController.open(deviceOptions))
    {
        cerr << _dbgtag << "Failed to open the device: " << controller << endl;

        //return false;
    }

    // Open a Cartesian controller

    if(!_deviceController.view(_armCartesianCtrl))
    {
        cerr << _dbgtag << "Failed to get a Cartesian view" << endl;
        // Cannot explore,
        // _exploreObjectValid = false;
        // return false;
    }

    // Remember the contorller context ID, restore when closing the port
    _armCartesianCtrl->storeContext(&_cartCtrlStartupIDstartupID);

    // Set the trajectory time
   //cout << "Trajectory time: " << trajectoryTime << endl;
    _armCartesianCtrl->setTrajTime(trajectoryTime);


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

    //_armCartesianController->setPosePriority("orientation");



    //////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////        Joint Control                     /////////////////
    //////////////////////////////////////////////////////////////////////////////////////

    yarp::os::Property optionsJnt;
    optionsJnt.put("device", "remote_controlboard");
    optionsJnt.put("local", "/" + moduleName + "/" + _whichHand + "_arm/joint");                 //local port names
    optionsJnt.put("remote", "/" + _robotName + "/" + _whichHand + "_arm");



    //cout << "Device options: " << optionsJnt.toString() << endl;

    if(!_deviceController_joint.open(optionsJnt))
    {
        cerr << _dbgtag << "Failed to open the device: " << "urgh" << endl; //systemParameters.getControllerType() << endl;
        // Cannot explore
      //  _exploreObjectValid = false;
       // return false;
    }

    //open an armcontroller_mode view
    if(!_deviceController_joint.view(_armJointModeCtrl))
    {
        cerr << _dbgtag << "Failed to open control mode view" << endl;
        // Cannot explore
        //_exploreObjectValid = false;
        //return false;
    }

    // Open an encoder view
    if(!_deviceController_joint.view(_armEncoders))
    {
        cerr << _dbgtag << "Failed to open Encoder view" << endl;
        // Cannot explore
       // _exploreObjectValid = false;
        //return false;
    }

    if(!_deviceController_joint.view(_armJointPositionCtrl))
    {
        cerr << _dbgtag << "Failed to open joint position controller view" << endl;
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

    t_controllerData ctrlData;
    ctrlData.whichHand = _whichHand;
    ctrlData.armJointModeCtrl = _armJointModeCtrl;
    ctrlData.armEncoder = _armEncoders;
    ctrlData.armJointPositionCtrl = _armJointPositionCtrl;


    /*
    if(_fingerEncoders.open("/" + _moduleName + "/" + _arm + "_hand/analog:i"))
    {
        Network::connect( "/" + _robotName + "/" + _arm + "_hand/analog:o",
                          "/" + _moduleName + "/" + _arm + "_hand/analog:i");
    }

    */
    FingerFactory fingerCreator;

    _thumb = fingerCreator.createFinger("thumb", ctrlData);
    _indexFinger = fingerCreator.createFinger("index", ctrlData);



}

}
