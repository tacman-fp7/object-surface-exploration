#include "finger.h"

#include <iostream>
#include <math.h>
#include <yarp/os/Network.h>

namespace objectExploration {

using std::cerr;
using std::endl;
using std::cout;
using yarp::os::Network;

bool Finger::setProximalAngle(double angle, double speed){
    return setAngle(_proximalJointIndex, angle, speed);
}

bool Finger::setDistalAngle(double angle, double speed){
    return setAngle(_distalJointIndex, angle, speed);
}

bool Finger::readEncoders(Vector &encoderValues){


}

bool Finger::getAngels(yarp::sig::Vector &angles){

    bool ret = true;


    //Vector joints;
    int nEncs;

    ret = ret && _armEncoder->getAxes(&nEncs);
    Vector encs(nEncs);
    if(! (ret = ret && _armEncoder->getEncoders(encs.data())))
    {
        cerr << _dbgtag << "Failed to read arm encoder data" << endl;
    }

    ret = ret && _iCubFinger->getChainJoints(encs, angles);

    return ret;

}

bool icubFinger::getAngels(Vector &angles){

    Vector fingerEncoders;
    bool ret;

    ret = readEncoders(fingerEncoders);
    getAngels(angles, fingerEncoders);

    return ret;
}

void icubFinger::getAngels(yarp::sig::Vector &angles, yarp::sig::Vector fingerEncoders){

    angles.resize(3);
    angles[0] = 90 * (1 - (fingerEncoders[0] - _minProximal) / (_maxProximal - _minProximal) );
    angles[1] = 90 * (1 - (fingerEncoders[1] - _minMiddle) / (_maxMiddle - _minMiddle) );
    angles[2] = 90 * (1 - (fingerEncoders[2] - _minDistal) / (_maxDistal - _minDistal) );
}


bool Finger::toArmPosition(Vector &fingertipPosition, Vector &retArmpPosition){


    bool ret = true;


    Vector joints;
    int nEncs;

    ret = ret && _armEncoder->getAxes(&nEncs);
    Vector encs(nEncs);
    if(! (ret = ret && _armEncoder->getEncoders(encs.data())))
    {
        cerr << _dbgtag << "Failed to read arm encoder data" << endl;
    }

    ret = ret && _iCubFinger->getChainJoints(encs, joints);

    //cout << fingerEncoders.toString() << endl;
    //Vector fingerEncoders;

    //ret = ret && readEncoders(fingerEncoders);




    // Replace the joins with the encoder readings

    //joints[1] = 90 * (1 - (fingerEncoders[0] - _minProximal) / (_maxProximal - _minProximal) );
    //joints[2] = 90 * (1 - (fingerEncoders[1] - _minMiddle) / (_maxMiddle - _minMiddle) );
    //joints[3] = 90 * (1 - (fingerEncoders[2] - _minDistal) / (_maxDistal - _minDistal) );

    //Convert the joints to radians.
    for (int j = 0; j < joints.size(); j++)
        joints[j] *= M_PI/180;

    yarp::sig::Matrix tipFrame = _iCubFinger->getH(joints);
    Vector tip_x = tipFrame.getCol(3);

    retArmpPosition.resize(3);
    retArmpPosition[0] = fingertipPosition[0] + tip_x[0];
    retArmpPosition[1] = fingertipPosition[1] + tip_x[1];
    retArmpPosition[2] = fingertipPosition[2] - tip_x[2];


    return ret;
}


bool icubFinger::toArmPosition(Vector &fingertipPosition, Vector &retArmpPosition){


    bool ret = true;


    Vector joints;
    int nEncs;

    ret = ret && _armEncoder->getAxes(&nEncs);
    Vector encs(nEncs);
    if(! (ret = ret && _armEncoder->getEncoders(encs.data())))
    {
        cerr << _dbgtag << "Failed to read arm encoder data" << endl;
    }

    ret = ret && _iCubFinger->getChainJoints(encs, joints);

    //cout << fingerEncoders.toString() << endl;
    Vector fingerEncoders;

    ret = ret && readEncoders(fingerEncoders);




    // Replace the joins with the encoder readings

    joints[1] = 90 * (1 - (fingerEncoders[0] - _minProximal) / (_maxProximal - _minProximal) );
    joints[2] = 90 * (1 - (fingerEncoders[1] - _minMiddle) / (_maxMiddle - _minMiddle) );
    joints[3] = 90 * (1 - (fingerEncoders[2] - _minDistal) / (_maxDistal - _minDistal) );

    //Convert the joints to radians.
    for (int j = 0; j < joints.size(); j++)
        joints[j] *= M_PI/180;

    yarp::sig::Matrix tipFrame = _iCubFinger->getH(joints);
    Vector tip_x = tipFrame.getCol(3);

    retArmpPosition.resize(3);
    retArmpPosition[0] = fingertipPosition[0] + tip_x[0];
    retArmpPosition[1] = fingertipPosition[1] + tip_x[1];
    retArmpPosition[2] = fingertipPosition[2] - tip_x[2];


    return ret;
}

bool Finger::getPosition(yarp::sig::Vector &position){

    // I am using an hybrid fingertip position forward kinematics. Fristly, I use the the actual encoder
    // data for the last three joints. Secondly, the behaviour of the icubFinger position estimation
    // is a little unpredictable, so I am transforming the fingertip position into the robot coordingates
    // myself.
    bool ret = true;
    Vector joints;

    int nEncs;

    position.clear();
    position.resize(3); //x,y, z position


    ret = ret && _armEncoder->getAxes(&nEncs);
    Vector encs(nEncs);
    if(! (ret = ret && _armEncoder->getEncoders(encs.data())))
    {
        cerr << _dbgtag << "Failed to read arm encoder data" << endl;
    }

    ret = ret && _iCubFinger->getChainJoints(encs, joints);

    //cout << fingerEncoders.toString() << endl;

    // TODO: This should be moved to readEncoders method
    //adjustMinMax(fingerEncoders[0], _minProximal, _maxProximal);
    //adjustMinMax(fingerEncoders[1], _minMiddle, _maxMiddle);
    //adjustMinMax(fingerEncoders[2], _minDistal, _maxDistal);



    // Replace the joins with the encoder readings

    //joints[1] = 90 * (1 - (fingerEncoders[0] - _minProximal) / (_maxProximal - _minProximal) );
    //joints[2] = 90 * (1 - (fingerEncoders[1] - _minMiddle) / (_maxMiddle - _minMiddle) );
    //joints[3] = 90 * (1 - (fingerEncoders[2] - _minDistal) / (_maxDistal - _minDistal) );

    //Convert the joints to radians.
    for (int j = 0; j < joints.size(); j++)
        joints[j] *= M_PI/180;

    yarp::sig::Matrix tipFrame = _iCubFinger->getH(joints);
    Vector tip_x = tipFrame.getCol(3); // Tip's position in the hand coordinate
    //cout << "TipX: " << tip_x.toString() << endl;
    Vector armPos, armOrient;
    _armCartesianCtrl->getPose(armPos, armOrient);

    // My own transformation
    yarp::sig::Matrix T_rotoTrans(4,4);
    T_rotoTrans = yarp::math::axis2dcm(armOrient);
    T_rotoTrans.setSubcol(armPos, 0,3);
    Vector retMat = yarp::math::operator *(T_rotoTrans, tip_x);
    position = retMat.subVector(0,2);
    //cout << "Finger position: " << position.toString()  << endl;

}

bool icubFinger::getPosition(yarp::sig::Vector &position){
    bool ret;

    Vector fingerEncoders;
    fingerEncoders.size(3);
    ret = readEncoders(fingerEncoders);

    ret = ret && getPosition(position, fingerEncoders);

    return ret;

}

bool Finger::getPosition(yarp::sig::Vector &position, yarp::sig::Vector &fingerEncoders){


    // Not using the fingerEncoders, it is there to make it compatible with the icubFinger class.
   return getPosition(position);


}


bool icubFinger::getPosition(yarp::sig::Vector &position, yarp::sig::Vector &fingerEncoders){


    // I am using an hybrid fingertip position forward kinematics. Fristly, I use the the actual encoder
    // data for the last three joints. Secondly, the behaviour of the icubFinger position estimation
    // is a little unpredictable, so I am transforming the fingertip position into the robot coordingates
    // myself.

    bool ret = true;
    Vector joints;

    int nEncs;

    position.clear();
    position.resize(3); //x,y, z position


    ret = ret && _armEncoder->getAxes(&nEncs);
    Vector encs(nEncs);
    if(! (ret = ret && _armEncoder->getEncoders(encs.data())))
    {
        cerr << _dbgtag << "Failed to read arm encoder data" << endl;
    }

    ret = ret && _iCubFinger->getChainJoints(encs, joints);

    //cout << fingerEncoders.toString() << endl;

    // TODO: This should be moved to readEncoders method
    adjustMinMax(fingerEncoders[0], _minProximal, _maxProximal);
    adjustMinMax(fingerEncoders[1], _minMiddle, _maxMiddle);
    adjustMinMax(fingerEncoders[2], _minDistal, _maxDistal);



    // Replace the joins with the encoder readings

    joints[1] = 90 * (1 - (fingerEncoders[0] - _minProximal) / (_maxProximal - _minProximal) );
    joints[2] = 90 * (1 - (fingerEncoders[1] - _minMiddle) / (_maxMiddle - _minMiddle) );
    joints[3] = 90 * (1 - (fingerEncoders[2] - _minDistal) / (_maxDistal - _minDistal) );

    //Convert the joints to radians.
    for (int j = 0; j < joints.size(); j++)
        joints[j] *= M_PI/180;

    yarp::sig::Matrix tipFrame = _iCubFinger->getH(joints);
    Vector tip_x = tipFrame.getCol(3); // Tip's position in the hand coordinate
    //cout << "TipX: " << tip_x.toString() << endl;
    Vector armPos, armOrient;
    _armCartesianCtrl->getPose(armPos, armOrient);

    // My own transformation
    yarp::sig::Matrix T_rotoTrans(4,4);
    T_rotoTrans = yarp::math::axis2dcm(armOrient);
    T_rotoTrans.setSubcol(armPos, 0,3);
    Vector retMat = yarp::math::operator *(T_rotoTrans, tip_x);
    position = retMat.subVector(0,2);
    //cout << "Finger position: " << position.toString()  << endl;
}

Finger::Finger(t_controllerData ctrlData){
    _armEncoder = ctrlData.armEncoder;
    _armJointModeCtrl = ctrlData.armJointModeCtrl;
    _armJointPositionCtrl = ctrlData.armJointPositionCtrl;
    _armCartesianCtrl = ctrlData.armCartesianCtrl;
    //_fingerEncoders = ctrlData.fingerEncoders;

    // /force-cop-estimator/left_index/force:o

    string forcePortName_remote = "/force-cop-estimator/" + ctrlData.whichHand + "_" +
            ctrlData.whichFinger + "/force:o";
    string copPortName_remote = "/force-cop-estimator/" + ctrlData.whichHand + "_" +
            ctrlData.whichFinger + "/cop:o";

    string forcePortName_local = "/object-exploration/" + ctrlData.whichHand + "_" +
            ctrlData.whichFinger + "/force:i";
    string copPortName_local = "/object-exploration/" + ctrlData.whichHand + "_" +
            ctrlData.whichFinger + "/cop:i";


    _contactForce_in.open(forcePortName_local);
    _contactCoP_in.open(copPortName_local);

    if(Network::exists(forcePortName_remote)){
        Network::connect(forcePortName_remote, forcePortName_local);
    }
    else{
       cerr << _dbgtag << "Port does not exist: " << forcePortName_remote << endl;
    }

    if(Network::exists(copPortName_remote)){
        Network::connect(copPortName_remote, copPortName_local);
    }
    else{
        cerr << _dbgtag << "Port does not exist: " << copPortName_remote << endl;
    }
}

icubFinger::icubFinger(t_controllerData ctrlData):Finger(ctrlData){
    _fingerEncoders = ctrlData.fingerEncoders;

}

bool Finger::getContactCoP(yarp::sig::Vector &contactCoP){
    int nPendingReads;
    contactCoP.resize(3);
    Bottle* cop;

    nPendingReads = _contactCoP_in.getPendingReads();
    for (int i = 0; i < nPendingReads; i++ ){
       cop = _contactCoP_in.read();
       cout << cop->toString() << endl;
    }

    if(!cop->isNull()){
        contactCoP(0)  = cop->get(0).asDouble();
        contactCoP(1) = cop->get(1).asDouble();
        contactCoP(2) = cop->get(2).asDouble();
        return true;
    }
    else{
       contactCoP(0) = 0;
       contactCoP(1) = 0;
       contactCoP(2) = 0;

       return false;
    }
}

double Finger::getContactForce(){
    int nPendingReads = _contactForce_in.getPendingReads();
    Bottle* contactForce;

    for (int i = 0; i < nPendingReads; i++){
        contactForce = _contactForce_in.read();
        cout << "contact force " << contactForce << endl;
    }

    if(!contactForce->isNull()){
        return   contactForce->get(0).asDouble();

    }
    else{
        return 0;
    }
}

void Finger::calibrate(){




}

/*bool Finger::readEncoders(Vector &encoderValues)
{

    bool ret = false;

    encoderValues.clear();
    encoderValues.resize(3);

    int nData = _fingerEncoders->getInputCount();
    Bottle *handEnc;

    for(int data = 0; data < nData; data++)
        handEnc = _fingerEncoders->read();

    if(!handEnc->isNull())
    {
        encoderValues[0] = handEnc->get(_proximalEncoderIndex).asDouble();
        encoderValues[1] = handEnc->get(_middleEncoderIndex).asDouble();
        encoderValues[2] = handEnc->get(_distalEncoderIndex).asDouble();
        ret = true;
    }

    //cout << "Encoders: " << encoderValues.toString() << endl;
    return ret;
}*/

bool Finger::setAngles(double proximalAngle, double speed){



    double distalAngle = 90 - proximalAngle;
    return setAngles(proximalAngle, distalAngle, speed);



}


void icubFinger::adjustMinMax(const double currentVal, double &min, double &max){


    if(currentVal > max)
        max = currentVal;
    if(currentVal < min)
        min = currentVal;


}

bool Finger::checkMotionDone(){
    bool retProximal;
    bool retDistal;

    if(!_armJointPositionCtrl->checkMotionDone(_proximalJointIndex, &retProximal))
    {
        std::cerr << _dbgtag << "CheckMotion failed on network comms" << std::endl;
        retProximal = true;
    }
    if(!_armJointPositionCtrl->checkMotionDone(_distalJointIndex, &retDistal))
    {
        std::cerr << _dbgtag << "CheckMotion failed on network comms" << std::endl;
        retDistal = true;
    }

    return (retProximal && retDistal);
}

bool Finger::setAngle(int jointIndex, double angle, double speed){

    bool ret = true;

    ret = ret && _armJointModeCtrl->setPositionMode(jointIndex);;
    ret = ret && _armJointPositionCtrl->setRefSpeed(jointIndex, speed);
    ret = ret && _armJointPositionCtrl->positionMove(jointIndex, angle);


    return ret;
}

bool Finger::setAngles(double proximal, double distal, double speed)
{
    bool ret = true;

    ret = ret && setAngle(_proximalJointIndex, proximal, speed);
    ret = ret && setAngle(_distalJointIndex, distal, speed);

    return ret;
}

bool Finger::open(){
   bool ret;

   ret = setAngle(_proximalJointIndex, 0, 30);
   ret = ret && setAngle(_distalJointIndex, 0, 30);

   // Wait until the finger is open
   while (!checkMotionDone()) {
      ;
   }

}

bool icubFinger::readEncoders(yarp::sig::Vector &encoderValues){
    bool ret = false;

    encoderValues.clear();
    encoderValues.resize(3);

    int nData = _fingerEncoders->getInputCount();
    Bottle *handEnc;

    for(int data = 0; data < nData; data++)
        handEnc = _fingerEncoders->read();

    if(!handEnc->isNull())
    {
        encoderValues[0] = handEnc->get(_proximalEncoderIndex).asDouble();
        encoderValues[1] = handEnc->get(_middleEncoderIndex).asDouble();
        encoderValues[2] = handEnc->get(_distalEncoderIndex).asDouble();
        ret = true;

        adjustMinMax(encoderValues[0], _minProximal, _maxProximal);
        adjustMinMax(encoderValues[1], _minMiddle, _maxMiddle);
        adjustMinMax(encoderValues[2], _minDistal, _maxDistal);
    }


    //cout << "Encoders: " << encoderValues.toString() << endl;
    return ret;
}

void icubFinger::calibrate(){

}

IndexFinger::IndexFinger(t_controllerData ctrlData):
    icubFinger(ctrlData){


    _iCubFinger = new iCub::iKin::iCubFinger(ctrlData.whichHand + "_index");

    // Joint indexes as defined in iCub
    _proximalJointIndex = INDEX_PROXIMAL;
    _distalJointIndex = INDEX_DISTAL;

    _proximalEncoderIndex = INDEX_PROXIMAL_ENCODER;
    _middleEncoderIndex = INDEX_MIDDLE_ENCODER;
    _distalEncoderIndex = INDEX_DISTAL_ENCODER;

    // TODO: Put it in a config file!
    _maxProximal = 235;
    _minProximal = 14;
    _maxMiddle = 215;
    _minMiddle = 20;
    _maxDistal = 250;
    _minDistal = 24;



}

SimIndexFinger::SimIndexFinger(t_controllerData ctrlData):
    Finger(ctrlData){


    _iCubFinger = new iCub::iKin::iCubFinger(ctrlData.whichHand + "_index");

    // Joint indexes as defined in iCub
    _proximalJointIndex = INDEX_PROXIMAL;
    _distalJointIndex = INDEX_DISTAL;

    _proximalEncoderIndex = INDEX_PROXIMAL_ENCODER;
    _middleEncoderIndex = INDEX_MIDDLE_ENCODER;
    _distalEncoderIndex = INDEX_DISTAL_ENCODER;


}

bool IndexFinger::setSynchroProximalAngle(double angle){

    double distal = 90-angle;
    return setAngles(angle, distal, 40);
}

bool SimIndexFinger::setSynchroProximalAngle(double angle){

    double distal = 90-angle;
    return setAngles(angle, distal, 40);
}

void IndexFinger::calibrate(){

    // Open the finger
    Finger::open();

    // Set the proximal to 0 and distal to 180
    setAngle(_proximalJointIndex, 0, 30);
    setAngle(_distalJointIndex, 180, 30);
    while(!checkMotionDone()){
        ;
    }



    // TODO: remove this for the simulation. There is not such data
    // available during simulation
    Bottle *fingerEnc;
    for(int i = 0; i < _fingerEncoders->getPendingReads(); i++)
        fingerEnc = _fingerEncoders->read();

    if(!fingerEnc->isNull())
    {
        _maxProximal = fingerEnc->get(_proximalEncoderIndex).asDouble();
        _minMiddle = fingerEnc->get(_middleEncoderIndex).asDouble();
        _minDistal = fingerEnc->get(_distalEncoderIndex).asDouble();
    }



    setAngle(_distalJointIndex, 0, 30);
    while(!checkMotionDone()){
        ;
    }

    setAngle(_proximalJointIndex, 90, 30);
    while(!checkMotionDone()){
        ;
    }



    // TODO: remove this for the simulation. There is not such data
    // available during simulation
    for(int i = 0; i < _fingerEncoders->getPendingReads(); i++)
        fingerEnc = _fingerEncoders->read();
    if(!fingerEnc->isNull())
    {
        _minProximal = fingerEnc->get(_proximalEncoderIndex).asDouble();
        _maxMiddle = fingerEnc->get(_middleEncoderIndex).asDouble();
        _maxDistal = fingerEnc->get(_distalEncoderIndex).asDouble();
    }

    setAngle(_proximalJointIndex, 0);
    while(!checkMotionDone()){
        ;
    }


    cout << "Calibration mins:" <<
            _minProximal << "\t" <<
            _minMiddle << "\t" <<
            _minDistal << "\t" << endl;

    cout << "Calibration maxes:" <<
            _maxProximal << "\t" <<
            _maxMiddle << "\t" <<
            _maxDistal << "\t" << endl;
}

bool IndexFinger::prepare(){

    bool ret = true;

    ret = ret && setAngle(_distalJointIndex, 20);
    ret = ret && setAngle(_proximalJointIndex, 20);

    return ret;

}

bool SimIndexFinger::prepare(){

    bool ret = true;

    ret = ret && setAngle(_distalJointIndex, 20);
    ret = ret && setAngle(_proximalJointIndex, 20);

    return ret;

}

Thumb::Thumb(t_controllerData ctrlData):
icubFinger(ctrlData){

    //Finger::Finger(armEncoder, armJointModeCtrl, armJointPositionCtrl);




    _iCubFinger = new iCub::iKin::iCubFinger(ctrlData.whichHand + "_thumb");

    // Joint indexes as defined in iCub
    _proximalJointIndex = THUMB_PROXIMAL;
    _distalJointIndex = THUMB_DISTAL;

    _proximalEncoderIndex = THUMB_PROXIMAL_ENCODER;
    _middleEncoderIndex = THUMB_MIDDLE_ENCODER;
    _distalEncoderIndex = THUMB_DISTAL_ENCODER;

    // TODO: Put it in a config file! Get the correct ones for the thumb!
    _maxProximal = 235;
    _minProximal = 14;
    _maxMiddle = 215;
    _minMiddle = 20;
    _maxDistal = 250;
    _minDistal = 24;

}

bool Thumb::prepare(){
    bool ret = true;

    ret = ret && Finger::setAngle(_proximalJointIndex, 0);
    ret = ret && Finger::setAngle(_distalJointIndex, 65);

    return ret;
}



SimThumb::SimThumb(t_controllerData ctrlData):
Finger(ctrlData){

    //Finger::Finger(armEncoder, armJointModeCtrl, armJointPositionCtrl);




    _iCubFinger = new iCub::iKin::iCubFinger(ctrlData.whichHand + "_thumb");

    // Joint indexes as defined in iCub
    _proximalJointIndex = THUMB_PROXIMAL;
    _distalJointIndex = THUMB_DISTAL;

    _proximalEncoderIndex = THUMB_PROXIMAL_ENCODER;
    _middleEncoderIndex = THUMB_MIDDLE_ENCODER;
    _distalEncoderIndex = THUMB_DISTAL_ENCODER;


}

bool SimThumb::prepare(){
    bool ret = true;

    ret = ret && Finger::setAngle(_proximalJointIndex, 0);
    ret = ret && Finger::setAngle(_distalJointIndex, 65);

    return ret;
}


}
