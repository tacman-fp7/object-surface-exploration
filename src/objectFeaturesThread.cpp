#include <objectFeaturesThread.h>
#include <yarp/os/Network.h>
#include <yarp/os/Value.h>
#include <yarp/os/Bottle.h>
#include <vector>
#include <iCub/iKin/iKinFwd.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <new>
#include <math.h>
#include <yarp/os/Time.h>


//#define M_PI   3.14159265358979323846264338328


namespace objectExploration
{

using yarp::os::Network;
using yarp::os::Value;
using yarp::os::Bottle;
using std::cout;
using std::endl;
using std::cerr;
using yarp::os::Mutex;


/*
 * 192 for hand data, where 1-60 are taxels of fingertips (12 each in this order:
 * index, middle, ring, little, thumb); 61-96 zeros; 97-144 palm taxels
 * (inside these, 108, 120, 132, and 140 are thermal pads ~ 0s); 145-192 zeros.
 */
void ObjectFeaturesThread::run()
{


    //cout << _dbgtag << "Still running!" << endl;
    ///// Read the tactile data ///////
    //Bottle* tactileData = _tactilePort.read(true); // Wait for data
    Bottle* contactForceCoP = _contactForceCoPPort.read(true); // Wait for data

    //// Read the corresponding arm position. //////
    //Bottle* armPose = _armPositionPort.read(true);

    if(contactForceCoP == NULL )
    {
        // TODO: figure out why it gets run when deleting the thread  <--- becuase it is was waiting for the tactile data
        cerr << _dbgtag << "Run: Null pointers!" << endl;
        return;
    }
    if(contactForceCoP->isNull())
    {
        cerr << "Did not receive tactile or arm data" << endl;
        return;
    }

    double encoderValue;
    _armPoseMutex.lock();
    _armCartesianCtrl->getPose(_armPosition, _armOrientation);
    /*for (int i = 0; i < 3; i++)
        _armPosition[i] = armPose->get(i).asDouble();
    for (int i = 3; i < 7; i++)
        _armOrientation[i-3] = armPose->get(i).asDouble();*/

    if(_armEncoder->getEncoder(_proximalJoint_index, &encoderValue))
        _proximalJointAngle = encoderValue;
    else
        cerr << _dbgtag << "Invalid proximal joint value." << endl;

    _armPoseMutex.unlock();

    //cout << _armPosition.toString() << " : " << _proximalJoint_index << " : " << encoderValue << endl;

    //cout << armPose->toString() << endl << endl;
    //cout << _armPosition.toString() << endl;
    //cout << _armOrientation.toString() << endl << endl;

    _tactileMutex.lock();
    _contactForce = contactForceCoP->get(0).asDouble(); // Contact force field is the first one
    _tactileMutex.unlock();


}

bool ObjectFeaturesThread::getArmPose(yarp::sig::Vector &pos, yarp::sig::Vector &orient)
{

    bool ret;
    _armPoseMutex.lock();
    ret = _armCartesianCtrl->getPose(_armPosition, _armOrientation);
    pos = _armPosition;
    orient = _armOrientation;
    _armPoseMutex.unlock();

    return ret;
}

bool ObjectFeaturesThread::fingerMovePosition(int joint, double angle, double speed)
{
    double dummy;
    _armJointPositionCtrl->getRefSpeed(joint, &dummy);
    //cout << "Ref speed" << dummy << endl;
    _armJointPositionCtrl->setRefSpeed(joint, speed);
    if(!_armJointPositionCtrl->positionMove(joint, angle)) //TODO: use the config file
    {
        cerr << _dbgtag << "Falied to move to the requsted positions." << endl;
    }
}



bool ObjectFeaturesThread::maintainContactPose()
{

    bool ret = true;

    openHand();

    // Put the thumb in position
    ret = fingerMovePosition(7, 12);
    ret = fingerMovePosition(9, 30);
    ret = fingerMovePosition(10, 170);

     /*
    Vector pos, orient;
    pos.resize(3);
    orient.resize(4);

   ret = _armCartesianCtrl->getPose(pos, orient);

    cout << "Pos: " << pos.toString() << "\tOrient: " << orient.toString() << endl;


    orient[3] += 0.4;

    if(ret){
        ret = _armCartesianCtrl->goToPose(pos,orient);
        ret = fingerMovePosition(11, 25);

    }

    */

    return ret;
}

bool ObjectFeaturesThread::prepHand()
{
    int numAxes;
    bool ret = true;

    if(! (ret = _armEncoder->getAxes( &numAxes)))
    {
        cerr << _dbgtag << "Could not read the number available arm axes." << endl;
        return false;
    }

    if(numAxes < 16)
    {
        cerr << _dbgtag << "Expected 16 axes, got" << numAxes << endl;
        return false;
    }


    ret = _armJointModeCtrl->setPositionMode(11);

    // Put the thumb in position
    ret = openHand();

    ret = fingerMovePosition(7, 0);
    ret = fingerMovePosition(9, 30);
    ret = fingerMovePosition(10, 170);
    ret = fingerMovePosition(11, 0);
    ret = fingerMovePosition(12, 55);






   return true;

}


bool ObjectFeaturesThread::openHand()
{
    int numAxes;


    if(!_armEncoder->getAxes( &numAxes))
    {
        cerr << _dbgtag << "Could not read the number available arm axes." << endl;
        return false;
    }

    if(numAxes < 16)
    {
        cerr << _dbgtag << "Expected 16 axes, got" << numAxes << endl;
        return false;
    }




    ////_armJointPositionCtrl->setPositionMode();
    ///// Quick fix /////
   /* if(!_armJointPositionCtrl->positionMove(7, 0)) //TODO: use the config file
    {
        cerr << _dbgtag << "Falied to move to the requsted positions." << endl;
    }


    if(!_armJointPositionCtrl->positionMove(8, 10)) //TODO: use the config file
    {
        cerr << _dbgtag << "Falied to move to the requsted positions." << endl;
    }*/

    for (int i=7; i < numAxes; i++)
    {
        if(!_armJointPositionCtrl->positionMove(i, 0))
        {
            cerr << _dbgtag << "Falied to move to the requsted positions." << endl;
        }
    }


   return true;
}


bool ObjectFeaturesThread::setProximalAngle(double angle){
    bool ret = false;
    if(_armJointPositionCtrl != NULL )
    {

       ret = _armJointModeCtrl->setPositionMode(11);
       ret = _armJointPositionCtrl->positionMove(_proximalJoint_index, angle);

    }
    else
    {
    std::cerr << _dbgtag << "The joint controller is not initialised" << std::endl;
    ret = false;
    }


    //maintainContactPose();

    return ret;
}


void ObjectFeaturesThread::adjustIndexFinger()
{

    _armPoseMutex.lock();
    double tempProximalAngle = _proximalJointAngle;
    _armPoseMutex.unlock();

    if(!_armJointPositionCtrl->positionMove(12, 90 - tempProximalAngle)) //TODO: use the config file
    {
        cerr << _dbgtag << "Falied to move to the requsted positions." << endl;
    }
}

bool ObjectFeaturesThread::openIndexFinger()
{
    bool ret;
    ret = _armJointPositionCtrl->positionMove(12,0);
    ret = _armJointPositionCtrl->positionMove(11,0);
    return ret;
}

void ObjectFeaturesThread::calibrateHand()
{




    _armJointPositionCtrl->positionMove(11,0);
    _armJointPositionCtrl->positionMove(12, 180);
    while(!checkOpenHandDone())
        ;
    yarp::os::Time::delay(5);
    cout << "Motion done";

    Bottle *fingerEnc;
    for(int i = 0; i < _fingerEncoders.getPendingReads(); i++)
        fingerEnc = _fingerEncoders.read();
    _maxIndexProximal = fingerEnc->get(3).asDouble();
    _minIndexMiddle = fingerEnc->get(4).asDouble();
    _minIndexDistal = fingerEnc->get(5).asDouble();



    _armJointPositionCtrl->positionMove(12, 0);
    while(!checkOpenHandDone())
        ;

    _armJointPositionCtrl->positionMove(11,90);

    while(!checkOpenHandDone())
        ;
    yarp::os::Time::delay(5);
    cout << "Motion done";

    for(int i = 0; i < _fingerEncoders.getPendingReads(); i++)
        fingerEnc = _fingerEncoders.read();

    _minIndexProximal = fingerEnc->get(3).asDouble();
    _maxIndexMiddle = fingerEnc->get(4).asDouble();
    _maxIndexDistal = fingerEnc->get(5).asDouble();

    _armJointPositionCtrl->positionMove(11,0);
    while(!checkOpenHandDone())
        ;

    cout << "Calibration mins:" <<
            _minIndexProximal << "\t" <<
            _minIndexMiddle << "\t" <<
            _minIndexDistal << "\t" << endl;

    cout << "Calibration maxes:" <<
            _maxIndexProximal << "\t" <<
            _maxIndexMiddle << "\t" <<
            _maxIndexDistal << "\t" << endl;

}

void ObjectFeaturesThread::adjustMinMax(const double currentVal, double &min, double &max)
{
if(currentVal > max)
    max = currentVal;
if(currentVal < min)
    min = currentVal;
}

bool ObjectFeaturesThread::getIndexFingerEncoder(yarp::sig::Vector &encoderValues)
{

    bool ret = false;

    encoderValues.clear();
    encoderValues.resize(3);

    int nData = _fingerEncoders.getInputCount();
     Bottle *handEnc;

    for(int data = 0; data < nData; data++)
        handEnc = _fingerEncoders.read();

    if(!handEnc->isNull())
    {
        encoderValues[0] = handEnc->get(3).asDouble();
        encoderValues[1] = handEnc->get(4).asDouble();
        encoderValues[2] = handEnc->get(5).asDouble();
        ret = true;
    }

    //cout << "Encoders: " << encoderValues.toString() << endl;
    return ret;
}

bool ObjectFeaturesThread::getIndexFingertipPosition(yarp::sig::Vector &position)
{

    bool ret = true;

    Vector fingerEncoders;
    fingerEncoders.size(3);
    ret = getIndexFingerEncoder(fingerEncoders);
    ret = ret && getIndexFingertipPosition(position, fingerEncoders);

    return ret;


}


bool ObjectFeaturesThread::getIndexFingertipPosition(yarp::sig::Vector &position, yarp::sig::Vector &fingerEncoders)
{

    // I am using an hybrid fingertip position forward kinematics. Fristly, I use the the actual encoder
    // data for the last three joints. Secondly, the behaviour of the icubFinger position estimation
    // is a little unpredictable, so I am transforming the fingertip position into the robot coordingates
    // myself.

    bool ret = true;
    Vector joints;
    iCub::iKin::iCubFinger finger(_whichFinger);
    int nEncs;

    position.clear();
    position.resize(3); //x,y, z position


    ret = ret && _armEncoder->getAxes(&nEncs);
    Vector encs(nEncs);
    if(! (ret = ret && _armEncoder->getEncoders(encs.data())))
    {
        cerr << _dbgtag << "Failed to read arm encoder data" << endl;
    }

    ret = ret && finger.getChainJoints(encs, joints);

    //cout << fingerEncoders.toString() << endl;

    adjustMinMax(fingerEncoders[0], _minIndexProximal, _maxIndexProximal);
    adjustMinMax(fingerEncoders[1], _minIndexMiddle, _maxIndexMiddle);
    adjustMinMax(fingerEncoders[2], _minIndexDistal, _maxIndexDistal);



    // Replace the joins with the encoder readings

    joints[1] = 90 * (1 - (fingerEncoders[0] - _minIndexProximal) / (_maxIndexProximal - _minIndexProximal) );
    joints[2] = 90 * (1 - (fingerEncoders[1] - _minIndexMiddle) / (_maxIndexMiddle - _minIndexMiddle) );
    joints[3] = 90 * (1 - (fingerEncoders[2] - _minIndexDistal) / (_maxIndexDistal - _minIndexDistal) );

    //cout << "Joints:" << joints.toString() << endl;

    //Convert the joints to radians.
    for (int j = 0; j < joints.size(); j++)
        joints[j] *= M_PI/180;



    yarp::sig::Matrix tipFrame = finger.getH(joints);

    Vector tip_x = tipFrame.getCol(3); // Tip's position in the hand coordinate



    Vector armPos, armOrient;

    _armCartesianCtrl->getPose(armPos, armOrient);


    // My own transformation

    yarp::sig::Matrix T_rotoTrans(4,4);

    T_rotoTrans = yarp::math::axis2dcm(armOrient);
    T_rotoTrans.setSubcol(armPos, 0,3);

    //cout << "TR" << endl << T_rotoTrans.toString() << endl;

    Vector retMat = yarp::math::operator *(T_rotoTrans, tip_x);

    cout << "F Pos: " << retMat.toString() << endl;


    position = retMat.subVector(0,2);

    cout << "Finger position: " << position.toString()  << endl;




}




bool ObjectFeaturesThread::changeOrient(double newOrient)
{
   Vector pos, orient;
   pos.resize(3);
   orient.resize(4);

   _armCartesianCtrl->getPose(pos, orient);

   orient[3] += newOrient;


   _armCartesianCtrl->goToPose(pos, orient);


}

bool ObjectFeaturesThread::getFingertipPose(yarp::sig::Vector &pos, yarp::sig::Vector &orient)
{
    bool ret = true;


    int nEncs;


    _armEncoder->getAxes(&nEncs);
    Vector encs(nEncs);
    if(! (ret = _armEncoder->getEncoders(encs.data())))
    {
        cerr << _dbgtag << "Failed to read arm encoder data" << endl;
    }



    //cout << "Encoder data" << encs.toString() << endl;

    Vector joints;
    iCub::iKin::iCubFinger finger(_whichFinger);

    //cout << "Finger: " << _whichFinger << endl;


    finger.getChainJoints(encs, joints);

    //cout << "Joints: " << joints.toString() << endl;

    //Convert the joints to radians.
    for (int j = 0; j < joints.size(); j++)
        joints[j] *= M_PI/180;



    yarp::sig::Matrix tipFrame = finger.getH(joints);


    //cout << "Tip frame:" << endl << tipFrame.toString() << endl;


    Vector tip_x = tipFrame.getCol(3);
    Vector tip_o = yarp::math::dcm2axis(tipFrame);



    //yarp::sig::Matrix h0 = finger.getH0();

    //cout << "H0" << endl << h0.toString() << endl;



    Vector armPos, armOrient;

    _armCartesianCtrl->getPose(armPos, armOrient);




    //////////////////
    //cout << "Tip" << tip_x.toString() << endl;
    // I should have a mutex here specsific for the carteria view!
    //_armPoseMutex.lock();

   if(!_armCartesianCtrl->attachTipFrame(tip_x, tip_o))
       ret = false;


    //yarp::os::Time::delay(1);

    if(!_armCartesianCtrl->getPose(pos, orient))
    {
        cerr << _dbgtag << "Failed to read the fingertip pose" << endl;
        ret = false;
    }

    if(!_armCartesianCtrl->removeTipFrame())
        ret = false;

    //_armPoseMutex.unlock();

    cout << "A Pos: " << armPos.toString() << endl;
    cout << "T Pos: " << tip_x.toString() << endl;

    cout << "F Pos: " << pos.toString() << endl;







    // My own transformation

    yarp::sig::Matrix T_rotoTrans(4,4);

    T_rotoTrans = yarp::math::axis2dcm(armOrient);
    T_rotoTrans.setSubcol(armPos, 0,3);

    //cout << "TR" << endl << T_rotoTrans.toString() << endl;

    Vector retMat = yarp::math::operator *(T_rotoTrans, tip_x);

    cout << "F Pos: " << retMat.toString() << endl;

    return ret;
}

/// Urgh

void ObjectFeaturesThread::writeToFingerController(std::string command)
{
    //Bottle rpcCommand, rpcResponse;
    //rpcCommand.addString(command);

    Bottle &message = _fingerController_port.prepare();
    message.clear();

    message.clear();

    //// Copied from Massimo
    if (!command.empty()){

        char *commandChar = new char[command.length() + 1];
        strcpy(commandChar,command.c_str());
        std::vector<string> wordList;
        char *target;

        target = strtok(commandChar," ");
        while(target != NULL){
            wordList.push_back(target);
            target = strtok(NULL," ");
        }

        for(size_t i = 0; i < wordList.size(); i++){
            message.add(yarp::os::Value(wordList[i]));
        }

    }


    _fingerController_port.writeStrict();

}

/////////// Accessor and mutators ///////////

Vector ObjectFeaturesThread::getPosition()
{ 
    _armPoseMutex.lock();
    Vector temp = _armPosition;
    _armPoseMutex.unlock();
    return temp;

}

bool ObjectFeaturesThread::getHomePose ( Vector& pos, Vector& orient )
{
    if(_homePose_isValid)
    {
        pos = _homePosition;
        orient = _homeOrientation;
    }
    else
        cerr << "Home pose is invalid" << endl;

    return _homePose_isValid;
}

void ObjectFeaturesThread::setHomePose ( Vector& pos, Vector& orient )
{
    _homePosition = pos;
    _homeOrientation = orient;
    _homePose_isValid = true;
    printPose(pos, orient);
}


void ObjectFeaturesThread::setEndPose ( Vector& pos, Vector& orient )
{
    _desiredEndPosition = pos;
    _desiredEndOrientation = orient;
    _desiredEndPose_isValid = true;
    printPose(pos, orient);
}

void ObjectFeaturesThread::setStartingPose ( Vector& pos, Vector& orient )
{
    cout << "Starting pose set" << endl;
    _desiredStartingPosition = pos;
    _desiredStartingOrientation = orient;
    _zMax = pos[2] + 0.05;
    _zMin = pos[2] - 0.05;
    cout << "Max: " << _zMax << " Min: " << _zMin << endl;

    _desiredStartingPose_isValid = true;
    printPose(pos, orient);

}

bool ObjectFeaturesThread::getDesiredEndPose ( Vector& pos, Vector& orient )
{
    if(_desiredEndPose_isValid)
    {
        pos = _desiredEndPosition;
        orient = _desiredEndOrientation;
    }
    return _desiredEndPose_isValid;
}

void ObjectFeaturesThread::setWayPoint ( Vector pos, Vector orient )
{


    _wayPoint_isValid = true;

    if(pos[0] >= 0)
    {
        cerr << _dbgtag << "Cannot have positive x-axis value" << endl;
        _wayPoint_isValid = false;
        return;
    }



    // TODO: in a config file
    double min = _zMin;
    double max =  _zMax;

    //if(_wayPointPos[2] == max || _wayPointPos[2] == min)
    //{
    //    _wayPoint_isValid = false;
    //    return;

    //}

    if(pos[2] < min)
    {

        cerr << _dbgtag << "Exceeded the z-axis limit ( " << min << " ): " << pos[2] << endl;
        pos[2] = min;

        if(_wayPointPos[2] == min)
            _wayPoint_isValid = false;
    }

    if(pos[2] > max)
    {
        cerr << _dbgtag << "Exceeded the z-axis limit ( " << max << " ): " << pos[2] << endl;
        pos[2] = max;

        if(_wayPointPos[2] == max)
            _wayPoint_isValid = false;
    }


    _wayPointPos = pos;
    _wayPointOrient = orient;

}


bool ObjectFeaturesThread::getWayPoint ( Vector& pos, Vector& orient, bool invalidateWayPoint )
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

bool ObjectFeaturesThread::getStartingPose ( Vector& pos, Vector& orient )
{
    if(_desiredStartingPose_isValid)
    {
        pos = _desiredStartingPosition;
        orient = _desiredStartingOrientation;
    }
    return _desiredStartingPose_isValid;

}

void ObjectFeaturesThread::printPose ( Vector& pos, Vector& orient )
{
    cout << "Position: " << pos.toString() << endl;
    cout << "Orientation: " << orient.toString() << endl;
}

double ObjectFeaturesThread::getContactForce()
{
    _tactileMutex.lock();
    double temp = _contactForce;
    _tactileMutex.unlock();

    return temp;
}


Vector ObjectFeaturesThread::getOrientation()
{
    _armPoseMutex.lock();
    Vector temp = _armOrientation;
    _armPoseMutex.unlock();
    return temp;
}



bool ObjectFeaturesThread::threadInit()
{
    yarp::os::RateThread::threadInit();

    bool ret = true;

  _dbgtag = "\n\nObjectFeaturesThread.cpp: ";

    /*   /////////////////// Connect to the tactile sensor port /////////////////
    if(!_tactilePort.open("/objectExploration/tactileSensors/" + _arm + "_hand")){
        ret = false;
        printf("Failed to open local tactile port\n");
    }

    Network::connect("/" + _robotName + "/skin/" + _arm + "_hand_comp",
                     "/objectExploration/tactileSensors/" + _arm + "_hand");
*/
    ////////////////// Connect to the forceCoP port ///////////////////////
    if(!_contactForceCoPPort.open("/" + _moduleName + "/skin/" + _arm + "_hand/" + _whichFinger + "/force-CoP:i"))
    {
        ret = false;
        cerr << _dbgtag << "Failed to open " << "/" << _moduleName << "/skin" << _arm << "_hand/" <<
                _whichFinger << "/force-CoP:i" << endl;
        _isExplorationValid = false;
//        printf("Failed to open local tactile port\n");
    }

    ///////////////////////////////////////
    //TODO: Change the incoming port!
    ////////////////////////////////////////
    if(!Network::connect("/force-reconstruction/" + _arm + "_index/force-CoP",
                     "/" + _moduleName + "/skin/" + _arm + "_hand/" + _whichFinger + "/force-CoP:i"))
    {
        _isExplorationValid = false;
        cerr << _dbgtag << "Failed to connect:" << endl;
        cerr << "/force-reconstruction/" + _arm + "_index/force-CoP" << " and" << endl;
        cerr << "/" + _moduleName + "/skin/" + _arm + "_hand/" + _whichFinger + "/force-CoP:i" << endl << endl;
    }

    /// Connect to the finger controller RPC port

    if(!_fingerController_port.open("/" + _moduleName + "/" + _arm + "_hand/" + _whichFinger + "/command:o"))
    {
        ret = false;
        cerr << _dbgtag << "Failed to open local fingerController port" << endl;
         _isExplorationValid = false;
    }

    if(!Network::connect("/" + _moduleName + "/" + _arm + "_hand/" + _whichFinger + "/command:o",
                     _fingerControllerPortName))
    {
        cerr << _dbgtag << "Failed to connect" << endl;
        cerr << "/" + _moduleName + "/" + _arm + "_hand/" + _whichFinger + "/command:o" << endl;
        cerr << _fingerControllerPortName << endl << endl;
         _isExplorationValid = false;
    }


    /////////////////////////////////////////
    if(_fingerEncoders.open("/" + _moduleName + "/" + _arm + "_hand/analog:i"))
    {
        Network::connect( "/" + _robotName + "/" + _arm + "_hand/analog:o",
                          "/" + _moduleName + "/" + _arm + "_hand/analog:i");
    }

    //calibrateFinger();

    ///////////////////////////////


    if(ret)
        cout << "Object features thread configured" << endl;
    else
        cerr << _dbgtag << "Error, object features thread failed during configuration" << endl;

    return ret;
}

void ObjectFeaturesThread::threadRelease()
{

}

ObjectFeaturesThread::~ObjectFeaturesThread()
{
    _contactForceCoPPort.close();
    //_armPositionPort.close();

}

void ObjectFeaturesThread::setArmController_cart(yarp::dev::ICartesianControl *cartesianCtrl)
{
    _armCartesianCtrl = cartesianCtrl;
}

void ObjectFeaturesThread::setArmController_jnt(yarp::dev::IEncoders *encoder, yarp::dev::IPositionControl *jointCtrl)
{

    _armEncoder = encoder;
    _armJointPositionCtrl = jointCtrl;
}

void ObjectFeaturesThread::setArmController_mode(yarp::dev::IControlMode2 *armJointCtrlmode)
{
    _armJointModeCtrl = armJointCtrlmode;
}

ObjectFeaturesThread::ObjectFeaturesThread ( int period, ResourceFinder rf ) : RateThread ( period )
{

    // Some sane and safe default values
    _trajectoryTime = 1; // By default take 5 seconds to complete a trajectory
    _maintainContactPeriod = 20;
    _readTactilePeriod = 20;
    _explorationThreadPeriod = 20;
    _isExplorationValid = true; // assume true,

    _desiredFroce = 0;

    _desiredStartingPose_isValid = false;
    _desiredStartingPosition.resize(3); // x,y,z position
    _desiredStartingOrientation.resize(4); // Axis angle

    _desiredEndPose_isValid = false;
    _desiredEndOrientation.resize(4);
    _desiredEndPosition.resize(3);

    _homePose_isValid = false;
    _homeOrientation.resize(4);
    _homePosition.resize(3);

    _wayPoint_isValid = false;
    _wayPointOrient.resize(4);
    _wayPointPos.resize(3);


    _armOrientation.resize(4);
    _armPosition.resize(3);


    _contactForce = 0;
    _rf = rf;

    _armCartesianCtrl = NULL;
    _armEncoder = NULL;
    _armJointPositionCtrl = NULL;

    _proximalJointAngle = 0;
    _proximalJoint_index = 11;

    _zMin = 0;
    _zMax = 0;

    // TODO: Put it in a config file!
    _maxIndexProximal = 235;
    _minIndexProximal = 14;
    _maxIndexMiddle = 215;
    _minIndexMiddle = 20;
    _maxIndexDistal = 250;
    _minIndexDistal = 24;

    ////////////// read the parameters from the config file ///////////////
    this->readParameters();


}

bool ObjectFeaturesThread::readParameters()
{

    _moduleName = _rf.check("moduleName", Value("object-exploration-server"),
                                 "module name (string)").asString().c_str();


    Bottle &robotParameters = _rf.findGroup("RobotParameters");
    if(!robotParameters.isNull()){
        // Read the arm configuration
        _arm = robotParameters.check("arm", Value("left")).asString();
        _robotName = robotParameters.check("robotName", Value("icubSim")).asString();
        _controller = robotParameters.check("controller", Value("Error")).asString();
        _controllerName = robotParameters.check("controllerName", Value("Error")).asString();
        _trajectoryTime = robotParameters.check("_trajectoryTime", Value(5)).asInt();
        _fingerControllerPortName = robotParameters.check("fingerControllerPortName",
                                                          Value("/plantIdentification/cmd:i")).asString();
        _whichFinger = robotParameters.check("whichFinger", Value("left_index")).asString();

    }


    Bottle& explorationParameters = _rf.findGroup("ExplorationParameters");
    Bottle* startingPose;
    Bottle* endPose;
    if(!explorationParameters.isNull())
    {
        _maintainContactPeriod = explorationParameters.check("maintainContactPeriod", Value(20)).asInt();
        _desiredFroce = explorationParameters.check("desiredFroce", Value(0)).asDouble();
        _readTactilePeriod = explorationParameters.check("readTactilePeriod", Value(20)).asInt();
        _explorationThreadPeriod = explorationParameters.check("explorationThreadPeriod", Value(20)).asInt();
        startingPose = explorationParameters.find("startingPose").asList();
        endPose = explorationParameters.find("endPose").asList();
    }


    //////// Initialise the starting pose //////
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

        //_desiredStartingPose_isValid = true;
    }

    if(endPose->size() < 7)
        cout << "End pose is invalid!" << endl;
    else
    {
        for(int i = 0; i < 3; i++)
            _desiredEndPosition[i] = endPose->get(i).asDouble();
        for(int i = 3; i < 7; i++)
            _desiredEndOrientation[i-3] = endPose->get(i).asDouble();
        _desiredEndPose_isValid = true;
    }

    cout << "Read the following configuration from the file:" << endl;
    cout << "Robot name: " << _robotName << endl;
    cout << "Arm: " << _arm << endl;
    cout << "Controller: " << _controller << endl;
    cout << "Controller name: " << _controllerName << endl;
    cout << "Trajectory time: " << _trajectoryTime << endl;

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

    cout << "Maintain contact thread period: " << _maintainContactPeriod << endl;
    cout << "Desired force: " << _desiredFroce << endl;
    cout << "Read tactile sensors thread period: " << _readTactilePeriod << endl;
    cout << "Exploration thread period: " << _explorationThreadPeriod << endl;
    cout << endl;

    if(_whichFinger.compare(_whichFinger.size()-5, 5, "index")==0)
        _proximalJoint_index = 11;
    else
        _proximalJoint_index = 9;

}

const string& ObjectFeaturesThread::getArm()
{
    return _arm;
}

const string& ObjectFeaturesThread::getControllerName()
{
    return _controllerName;
}

const string& ObjectFeaturesThread::getControllerType()
{
    return _controller;
}

const string& ObjectFeaturesThread::getRobotName()
{
    return _robotName;
}

const int& ObjectFeaturesThread::getTrajectoryTime()
{
    return _trajectoryTime;
}

const int& ObjectFeaturesThread::getExplorationThreadPeriod()
{
    return _explorationThreadPeriod;
}

const int& ObjectFeaturesThread::getMaintainContactPeriod()
{
    return _maintainContactPeriod;
}

const double& ObjectFeaturesThread::getDesiredForce()
{
    return _desiredFroce;
}

} // namespace objectExploration
