#include "hand.h"
#include <iostream>
#include <yarp/os/Value.h>

namespace objectExploration {
using std::cerr;
using std::endl;
using std::cout;

using yarp::os::Value;

Hand::Hand(ResourceFinder rf){

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

bool Hand::setAbduction(double angle, double speed){

    _armJointPositionCtrl->setRefSpeed(ABDUCTION, speed);
    if(!_armJointPositionCtrl->positionMove(ABDUCTION, angle))
    {
        cerr << _dbgtag << "Falied to move to the requsted positions." << endl;
    }
}

void Hand::configure(yarp::os::ResourceFinder rf){






    Bottle &robotParameters = rf.findGroup("RobotParameters");
    if(robotParameters.isNull()){
        return;
    }

    // Read the arm configuration
    string arm = robotParameters.check("arm", Value("left")).asString();
    string robotName = robotParameters.check("robotName", Value("icubSim")).asString();
    string controller = robotParameters.check("controller", Value("Error")).asString();
    string controllerName = robotParameters.check("controllerName", Value("Error")).asString();
    int trajectoryTime = robotParameters.check("trajectoryTime", Value(5)).asInt();

    string whichFinger = robotParameters.check("whichFinger", Value("index")).asString();
    string moduleName = "object-exploration" ;

    cout << "Read the following configuration from the file:" << endl;
    cout << "Robot name: " << robotName << endl;
    cout << "Arm: " << arm << endl;
    cout << "Controller: " << controller << endl;
    cout << "Controller name: " << controllerName << endl;
    cout << "Trajectory time: " << trajectoryTime << endl;



    ///////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////// Configure the Cartersian controller ////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////

    yarp::os::Property deviceOptions;
    deviceOptions.put("device", controller);
    deviceOptions.put("local", "/" + moduleName
                      + "/" + controllerName + "/" + arm + "_arm");
    deviceOptions.put("remote", "/" + robotName
                      + "/" + controllerName + "/" + arm + "_arm");

    cout << "Device options: " << deviceOptions.toString() << endl;

    if(!_deviceController.open(deviceOptions))
    {
        cerr << _dbgtag << "Failed to open the device: " << controller << endl;

        //return false;
    }

    // Open a Cartesian controller

    if(!_deviceController.view(_armCartesianController))
    {
        cerr << _dbgtag << "Failed to get a Cartesian view" << endl;
        // Cannot explore,
        // _exploreObjectValid = false;
        // return false;
    }

    // Remember the contorller context ID, restore when closing the port
    _armCartesianController->storeContext(&_cartCtrlStartupIDstartupID);

    // Set the trajectory time
    cout << "Trajectory time: " << trajectoryTime << endl;
    _armCartesianController->setTrajTime(trajectoryTime);


    // Enable the torso movement

    Vector curDof;
    _armCartesianController->getDOF(curDof);
    //cout<<"["<<curDof.toString()<<"]"<<endl;  // [0 0 0 1 1 1 1 1 1 1] will be printed out
    Vector newDof(3);
    newDof[0]=1;    // torso pitch: 1 => enable
    newDof[1]=2;    // torso roll:  2 => skip
    newDof[2]=1;    // torso yaw:   1 => enable
    _armCartesianController->setDOF(newDof,curDof);
    //cout<<"["<<curDof.toString()<<"]"<<endl;  // [1 0 1 1 1 1 1 1 1 1] will be printed out

    //_armCartesianController->setPosePriority("orientation");



    //////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////        Joint Control                     /////////////////
    //////////////////////////////////////////////////////////////////////////////////////

    yarp::os::Property optionsJnt;
    optionsJnt.put("device", "remote_controlboard");
    optionsJnt.put("local", "/" + moduleName + "/" + arm + "_arm/joint");                 //local port names
    optionsJnt.put("remote", "/" + robotName
                   + "/" + arm + "_arm");



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

    if(!_deviceController_joint.view(_armJointPositionController))
    {
        cerr << _dbgtag << "Failed to open joint position controller view" << endl;
        //_exploreObjectValid = false;
        //return false;
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



    t_controllerData ctrlData;
    ctrlData.whichHand = arm;
    ctrlData.armJointModeCtrl = _armJointModeCtrl;
    ctrlData.armEncoder = _armEncoders;
    ctrlData.armJointPositionCtrl = _armJointPositionController;

    FingerFactory fingerCreator;

    _thumb = fingerCreator.createFinger("thumb", ctrlData);
    _indexFinger = fingerCreator.createFinger("index", ctrlData);



}

}
