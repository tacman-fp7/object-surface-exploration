#include "finger.h"

#include <iostream>
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



void Finger::logTactileCoP(){

    if(!_tactileDataCompFile.is_open()){
        // I am assuming if one is not open all are not open
        std::string fileName;
        fileName = _whichFinger + "_tactileComp.csv";
        _tactileDataCompFile.open(fileName.c_str());

        fileName = _whichFinger + "_tactileRaw.csv";
        _tactileDataRawFile.open(fileName.c_str());
        fileName = _whichFinger + "_cop.csv";
        _copFile.open(fileName.c_str());
    }

    Vector data;
    getContactCoP(data);
    _copFile << data[0] << ", " << data[1] << ", " << data[2] << std::endl;

    getTactileDataComp(data);
    for (int i = 0; i < 11; i++){
        _tactileDataCompFile << data[i] << ", ";
    }
    _tactileDataCompFile << data[11] << std::endl;

    getTactileDataRaw(data);
    for (int i = 0; i < 11; i++){
        _tactileDataRawFile << data[i] << ", ";
    }
    _tactileDataRawFile << data[11] << std::endl;

}

bool Finger::prepare(){
    cerr << "Prepare not implemented for this finger: " << _whichFinger << endl;
    return false;
}

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
    encoderValues.zero();
    return true;

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



bool Finger::getPositionHandFrame(yarp::sig::Vector &position, yarp::sig::Vector &fingerEncoders){

    cerr << _dbgtag << "get position has not been implemented for this finger." << endl;
    return false;

}

bool Finger::getPositionHandFrameCorrected(yarp::sig::Vector &position, yarp::sig::Vector &fingerEncoders){
    cerr << _dbgtag << "get position in hand frame has not been implemented for this finger: " << this->getFingerName() << endl;
    return false;
}

double Finger::getContactForceThreshold(){
    return _contactForceThreshold;
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



    //Convert the joints to radians.
    for (int j = 0; j < joints.size(); j++)
        joints[j] *= DEG2RAD;

    yarp::sig::Matrix tipFrame = _iCubFinger->getH(joints);
    Vector tip_x = tipFrame.getCol(3);

    retArmpPosition.resize(3);
    if(_whichHand.compare("left") == 0){
        retArmpPosition[0] = fingertipPosition[0] + tip_x[0];
        retArmpPosition[1] = fingertipPosition[1] + tip_x[1];
        retArmpPosition[2] = fingertipPosition[2] - tip_x[2];
    }else if(_whichHand.compare("right") == 0){

        retArmpPosition[0] = fingertipPosition[0] + tip_x[0];
        retArmpPosition[1] = fingertipPosition[1] - tip_x[1];
        retArmpPosition[2] = fingertipPosition[2] + tip_x[2];
    }


    return ret;
}






bool Finger::getPositionHandFrame(yarp::sig::Vector &position){


    bool ret;

    Vector fingerEncoders;
    fingerEncoders.resize(3);
    ret = readEncoders(fingerEncoders);

    ret = ret && getPositionHandFrame(position, fingerEncoders);

    return ret;


}

bool Finger::getPositionHandFrameCorrected(yarp::sig::Vector &position){
    cout << "not implemented" << endl;
}




bool Finger::getPositionCorrected(yarp::sig::Vector &position){

    bool ret = true;
    Vector tip_x;

    getPositionHandFrameCorrected(tip_x);
    tip_x.resize(4);
    tip_x[3] = 1.0;
    //    cout << "TipX: " << tip_x.toString() << endl;

    Vector armPos, armOrient;
    _armCartesianCtrl->getPose(armPos, armOrient);

    // My own transformation
    yarp::sig::Matrix T_rotoTrans(4,4);
    T_rotoTrans = yarp::math::axis2dcm(armOrient);
    T_rotoTrans.setSubcol(armPos, 0,3);
    Vector retMat = yarp::math::operator *(T_rotoTrans, tip_x);
    position = retMat.subVector(0,2);

    return ret;

}

bool Finger::getPositionCorrected(yarp::sig::Vector &position, yarp::sig::Vector &fingerEncoders){

    bool ret = true;
    Vector tip_x;

    getPositionHandFrameCorrected(tip_x, fingerEncoders);

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

    return ret;

}

bool Finger::getPosition(yarp::sig::Vector &position){
    bool ret;
    Vector fingerEncoders;
    fingerEncoders.resize(3);
    ret = readEncoders(fingerEncoders);
    ret = ret && getPosition(position, fingerEncoders);
    return ret;
}

bool Finger::getPosition(yarp::sig::Vector &position, yarp::sig::Vector &fingerEncoders){



    // I am using an hybrid fingertip position forward kinematics. Fristly, I use the the actual encoder
    // data for the last three joints. Secondly, the behaviour of the icubFinger position estimation
    // is a little unpredictable, so I am transforming the fingertip position into the robot coordingates
    // myself.
    bool ret = true;
    Vector tip_x;

    getPositionHandFrame(tip_x, fingerEncoders);

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

    return ret;



}


string Finger::getFingerName(){
    return _whichFinger;
}

Finger::~Finger(){
    _copFile.close();
    _tactileDataCompFile.close();
    _tactileDataRawFile.close();
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
    _whichHand = ctrlData.whichHand;
    _rawTactileData_in = ctrlData.rawTactileData_in;
    _tactileDataComp_in = ctrlData.tactileDataComp_in;
    _armControlLimits = ctrlData.armControlLimits;

    configure(ctrlData.whichFinger, ctrlData.rf);
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

void Finger::configure(std::string fingerName, yarp::os::ResourceFinder *rf){


    Bottle fingerParameters = rf->findGroup(fingerName);
    Bottle* encoderLimits;

    if(!fingerParameters.isNull()){
        _contactForceThreshold = fingerParameters.check("contactForceThreshold", yarp::os::Value(10)).asDouble();

        encoderLimits = fingerParameters.find("encoderLimits").asList();


    }

    // Oder of check is important to not sigfault
    if(encoderLimits == NULL || encoderLimits->size() < 6  ){
        cout << _dbgtag << "encoder limits are invalid; using default values." << endl;
        _maxProximal = 250;
        _minProximal = 0;
        _maxMiddle = 250;
        _minMiddle = 0;
        _maxDistal = 250;
        _minDistal = 0;
    }
    else{

        _maxProximal = encoderLimits->get(0).asInt();
        _minProximal = encoderLimits->get(1).asInt();
        _maxMiddle = encoderLimits->get(2).asInt();
        _minMiddle = encoderLimits->get(3).asInt();
        _maxDistal = encoderLimits->get(4).asInt();
        _minDistal = encoderLimits->get(5).asInt();



    }






}

bool Finger::getPositionCoPAdjusted(yarp::sig::Vector &position){
    // To implemented
    cout << "Not yet implemented" << endl;
}

void Finger::setContactForceThreshold(const double threshold){
    _contactForceThreshold = threshold;
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

bool Finger::calibrate(){

    return true;


}



bool Finger::setAngles(double proximalAngle, double speed){



    double distalAngle = 90 - proximalAngle;
    return setAngles(proximalAngle, distalAngle, speed);



}




bool Finger::checkMotionDone(){
    bool retProximal = false;
    bool retDistal = false;

    if(!_armJointPositionCtrl->checkMotionDone(_proximalJointIndex, &retProximal))
    {
        std::cerr << _dbgtag << "CheckMotion failed!" << std::endl;
        retProximal = true;
    }
    if(!_armJointPositionCtrl->checkMotionDone(_distalJointIndex, &retDistal))
    {
        std::cerr << _dbgtag << "CheckMotion failed!" << std::endl;
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

    //ret =  _armJointModeCtrl->setPositionMode(jointIndex);;
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
    return ret;
}


void Finger::checkMinMax(double &min, double &max){
    double temp;
    // Chec if we need to swap
    if(min > max){
        temp = min;
        min = max;
        max = temp;
        cerr << _dbgtag << "MinMax swapped!" << endl;
    }
}




}
