#include "finger.h"

#include <iostream>
#include <math.h>
#include <yarp/os/Network.h>
#include <stdexcept>
#include <deque>


namespace objectExploration {

using std::cerr;
using std::endl;
using std::cout;
using yarp::os::Network;
using yarp::dev::IControlLimits;
using std::deque;

void Finger::alignJointsBounds(){

    deque<IControlLimits*> limits;
    limits.push_back(_armControlLimits);
    _iCubFinger->alignJointsBounds(limits);
}

bool Finger::setProximalAngle(double angle, double speed){
    _curProximalAngle = angle;
    return setAngle(_proximalJointIndex, angle, speed);
}

bool Finger::setDistalAngle(double angle, double speed){
    _curDistalAngle = angle;
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

    cout << encs.toString() << endl;
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
    fingerEncoders.resize(3);
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


    //cout << encs.toString() << endl;

    ret = ret && _iCubFinger->getChainJoints(encs, joints);


    if(ret == false){
        cout << "failed to get chain joints" << endl;
        return false;
    }


    // Replace the joins with the encoder readings

    joints[1] = 90 * (1 - (fingerEncoders[0] - _minProximal) / (_maxProximal - _minProximal) );
    joints[2] = 90 * (1 - (fingerEncoders[1] - _minMiddle) / (_maxMiddle - _minMiddle) );
    joints[3] = 90 * (1 - (fingerEncoders[2] - _minDistal) / (_maxDistal - _minDistal) );

    //cout << joints.size() << endl;
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
    _prevContactForce = 0;
    _isForceValid = false;
    _isCoPValid = false;
    _isActiveTaxelValid = false;
    _whichFinger = ctrlData.whichFinger;
    _rawTactileData_in = ctrlData.rawTactileData_in;
    _armControlLimits = ctrlData.armControlLimits;

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







    if(!_contactForce_in.open(forcePortName_local)){
        cerr << _dbgtag << "could not open the local force port: " << forcePortName_local << endl;
        throw std::runtime_error(_dbgtag + "could not open the local force port: " + forcePortName_local + "\n");
    }

    if(!_contactCoP_in.open(copPortName_local)){

        cerr << _dbgtag << "could not open the local CoP port: " << copPortName_local << endl;
        throw std::runtime_error(_dbgtag + "could not open the local CoP port: " + copPortName_local + "\n");

    }

    if(Network::exists(forcePortName_remote)){
        if(Network::connect(forcePortName_remote, forcePortName_local)){
            _isForceValid = true;
        }
    }
    else{
        cerr << _dbgtag << "Port does not exist: " << forcePortName_remote << endl;
    }

    if(Network::exists(copPortName_remote)){
        if(Network::connect(copPortName_remote, copPortName_local)){
            _isCoPValid = true;
        }
    }
    else{
        cerr << _dbgtag << "Port does not exist: " << copPortName_remote << endl;
    }

    _fingerControlPort_out.open("/object-exploration/" + ctrlData.whichHand + "_hand/" + _whichFinger + "/control:o");
}

bool Finger::hasForceCoP(){
    return (_isForceValid && _isCoPValid);
}

icubFinger::icubFinger(t_controllerData ctrlData):Finger(ctrlData){
    _fingerEncoders = ctrlData.fingerEncoders;


}

bool Finger::getPositionCoPAdjusted(yarp::sig::Vector &position){
    // To implemented
    cout << "Not yet implemented" << endl;
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
    Bottle* contactForce = NULL;

    for (int i = 0; i < nPendingReads; i++){
        contactForce = _contactForce_in.read();
        //cout << "contact force " << contactForce->get(0).asDouble() << endl;
    }

    if(contactForce != NULL){
        _prevContactForce = contactForce->get(0).asDouble();
        return   _prevContactForce;

    }
    else{
        return _prevContactForce;
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
    bool retProximal = false;
    bool retDistal = false;

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

    Bottle& msg = _fingerControlPort_out.prepare();
    msg.clear();
    msg.addDouble(_curProximalAngle);
    msg.addDouble(_curDistalAngle);
    _fingerControlPort_out.writeStrict();

    ret =  _armJointModeCtrl->setPositionMode(jointIndex);;
    ret =  _armJointPositionCtrl->setRefSpeed(jointIndex, speed);
    ret =  _armJointPositionCtrl->positionMove(jointIndex, angle);


    return ret;
}

bool Finger::setAngles(double proximal, double distal, double speed)
{
    bool ret = true;
    _curProximalAngle = proximal;
    _curDistalAngle = distal;

    ret = setAngle(_proximalJointIndex, proximal, speed);
    ret = setAngle(_distalJointIndex, distal, speed);

    return ret;
}

bool Finger::open(){
    bool ret;

    _curProximalAngle = 0;
    _curDistalAngle = 0;
    ret = setAngle(_proximalJointIndex, _curProximalAngle, 30);
    ret = ret && setAngle(_distalJointIndex, _curDistalAngle, 30);

    // Wait until the finger is open
    while (!checkMotionDone()) {
        ;
    }

}

bool icubFinger::readEncoders(yarp::sig::Vector &encoderValues){
    bool ret = false;


    encoderValues.resize(3);
    encoderValues.zero();

    int pendingReads = _fingerEncoders->getPendingReads();
    Bottle *handEnc = NULL;

    for(int i = 0; i <= pendingReads; i++){
        handEnc = _fingerEncoders->read();
    }

    if(handEnc != NULL)
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

void IndexFinger::getRawTactileData(yarp::sig::Vector& rawTactileData){

    // Read from the port
    rawTactileData.resize(12);
    rawTactileData.zero();

    Bottle* tactileData;
    tactileData = _rawTactileData_in->read();

    //cout << tactileData->toString() << endl;
    int startIndex = 0;
    if(tactileData != NULL){
        for (int i = startIndex; i < 12; i++)
        {
            rawTactileData[i - startIndex] = tactileData->get(i).asDouble();
        }
    }
}

IndexFinger::IndexFinger(t_controllerData ctrlData):
    icubFinger(ctrlData){


    _iCubFinger = new iCub::iKin::iCubFinger(ctrlData.whichHand + "_index");
    alignJointsBounds();

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
    alignJointsBounds();
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

bool MiddleFinger::setSynchroProximalAngle(double angle){

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
    for(int i = 0; i <= _fingerEncoders->getPendingReads(); i++)
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

    //setSynchroProximalAngle(0);
    ret = ret && setAngle(_distalJointIndex, 0);
    ret = ret && setAngle(_proximalJointIndex, 0);

    return ret;

}

bool MiddleFinger::prepare(){

    bool ret = true;

    //setSynchroProximalAngle(0);
    ret = ret && setAngle(_distalJointIndex, 0);
    ret = ret && setAngle(_proximalJointIndex, 0);

    return ret;

}

bool SimIndexFinger::prepare(){

    bool ret = true;

    _curProximalAngle = 0;
    _curDistalAngle = 90;

    ret = ret && setAngle(_proximalJointIndex, _curProximalAngle);
    ret = ret && setAngle(_distalJointIndex, _curDistalAngle);

    return ret;

}


///////////////////////Middle Fingr///////////////////

MiddleFinger::MiddleFinger(t_controllerData ctrlData):
    icubFinger(ctrlData){

    _iCubFinger = new iCub::iKin::iCubFinger(ctrlData.whichHand + "_middle");
    alignJointsBounds();
    // Joint indexes as defined in iCub
    _proximalJointIndex = MIDDLE_PROXIMAL;
    _distalJointIndex = MIDDLE_DISTAL;

    _proximalEncoderIndex = MIDDLE_PROXIMAL_ENCODER;
    _middleEncoderIndex = MIDDLE_MIDDLE_ENCODER;
    _distalEncoderIndex = MIDDLE_DISTAL_ENCODER;

    // TODO: Put it in a config file!
    _maxProximal = 235;
    _minProximal = 14;
    _maxMiddle = 215;
    _minMiddle = 20;
    _maxDistal = 250;
    _minDistal = 24;

}

// Slightly different from the index finger. It is not affected by the abduction
bool MiddleFinger::getPosition(yarp::sig::Vector &position, yarp::sig::Vector &fingerEncoders){


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


    //cout << encs.toString() << endl;
    //cout << fingerEncoders.toString() << endl;
    ret = ret && _iCubFinger->getChainJoints(encs, joints);


    if(ret == false){
        cout << "failed to get chain joints" << endl;
        return false;
    }


    //std::cout << "From iCubFinger: " << joints.toString() << endl;
    //This is where it is different from the index finger

    joints[0] = 90 * (1 - (fingerEncoders[0] - _minProximal) / (_maxProximal - _minProximal) );
    joints[1] = 90 * (1 - (fingerEncoders[1] - _minMiddle) / (_maxMiddle - _minMiddle) );
    joints[2] = 90 * (1 - (fingerEncoders[2] - _minDistal) / (_maxDistal - _minDistal) );

    //cout << "From mycalculat: " << joints.toString() << endl;
    //cout << joints.size() << endl;
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

/// Ring and Little Finger joint class ////


RingAndLittleFingers::RingAndLittleFingers(t_controllerData ctrlData): icubFinger(ctrlData){
    _proximalEncoderIndex = PINKY;
    _distalJointIndex = PINKY;


}

bool  RingAndLittleFingers::setAngles(double proximal, double distal, double speed){
    return Finger::setAngle(PINKY, proximal, speed);
}

bool RingAndLittleFingers::setAngles(double proximal, double speed){
    return Finger::setAngle(PINKY, proximal, speed);
}

bool RingAndLittleFingers::setProximalAngle(double angle, double speed){
    return Finger::setAngle(PINKY, angle, speed);
}

bool RingAndLittleFingers::setDistalAngle(double angle, double speed){
    return Finger::setAngle(PINKY, angle, speed);
}

bool RingAndLittleFingers::setSynchroProximalAngle(double proximal){
    return setProximalAngle(PINKY, proximal);
}

RingFinger::RingFinger(t_controllerData ctrlData): RingAndLittleFingers(ctrlData){

    _iCubFinger = new iCub::iKin::iCubFinger(ctrlData.whichHand + "_ring");
    alignJointsBounds();

    _proximalEncoderIndex = RING_PROXIMAL_ENCODER;
    _middleEncoderIndex = RING_MIDDLE_ENCODER;
    _distalEncoderIndex = RING_DISTAL_ENCODER;

    _proximalJointIndex = PINKY;
    _distalJointIndex = PINKY;
    // TODO: check if I can get them from the IControlLimits

    _maxProximal = 235;
    _minProximal = 14;
    _maxMiddle = 215;
    _minMiddle = 20;
    _maxDistal = 250;
    _minDistal = 24;
}

LittleFinger::LittleFinger(t_controllerData ctrlData): RingAndLittleFingers(ctrlData){
    _iCubFinger = new iCub::iKin::iCubFinger(ctrlData.whichHand + "_little");
    alignJointsBounds();

    _proximalEncoderIndex = LITTLE_PROXIMAL_ENCODER;
    _middleEncoderIndex = LITTLE_MIDDLE_ENCODER;
    _distalEncoderIndex = LITTLE_DISTAL_ENCODER;

    _proximalJointIndex = PINKY;
    _distalJointIndex = PINKY;
    // TODO: check if I can get them from the IControlLimits

    _maxProximal = 235;
    _minProximal = 14;
    _maxMiddle = 215;
    _minMiddle = 20;
    _maxDistal = 250;
    _minDistal = 24;
}

/*bool icubFinger::calibrate2(){
    _armControlLimits->getLimits(_proximalJointIndex, &_minProximal, &_maxProximal);
    _armControlLimits->getLimits(_distalJointIndex, &_minDistal, &_maxDistal);
}*/

void icubFinger::printJointLimits(){

    std::cout << "Min proximal: " << _minProximal << ",\tMax proximal" << _maxProximal << std::endl;
    std::cout << "Min middle: " << _minMiddle << ",\tMax middle" << _maxMiddle << std::endl;
    std::cout << "Min distal: " <<  _minDistal << ",\tMax distal" << _maxDistal << std::endl;
}

/////////////////////
/// \brief Thumb::Thumb
/// \param ctrlData
///
Thumb::Thumb(t_controllerData ctrlData):
    icubFinger(ctrlData){





    _iCubFinger = new iCub::iKin::iCubFinger(ctrlData.whichHand + "_thumb");
    alignJointsBounds();
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

    _curProximalAngle = 0;
    _curDistalAngle = 65;
    ret = ret && Finger::setAngle(_proximalJointIndex, _curProximalAngle);
    ret = ret && Finger::setAngle(_distalJointIndex, _curDistalAngle);

    return ret;
}



SimThumb::SimThumb(t_controllerData ctrlData):
    Finger(ctrlData){

    //Finger::Finger(armEncoder, armJointModeCtrl, armJointPositionCtrl);




    _iCubFinger = new iCub::iKin::iCubFinger(ctrlData.whichHand + "_thumb");
    alignJointsBounds();
    // Joint indexes as defined in iCub
    _proximalJointIndex = THUMB_PROXIMAL;
    _distalJointIndex = THUMB_DISTAL;

    _proximalEncoderIndex = THUMB_PROXIMAL_ENCODER;
    _middleEncoderIndex = THUMB_MIDDLE_ENCODER;
    _distalEncoderIndex = THUMB_DISTAL_ENCODER;


}

bool SimThumb::prepare(){
    bool ret = true;

    _curProximalAngle = 0;
    _curDistalAngle = 65;
    ret = ret && Finger::setAngle(_proximalJointIndex, _curProximalAngle);
    ret = ret && Finger::setAngle(_distalJointIndex, _curDistalAngle);

    return ret;
}


}
