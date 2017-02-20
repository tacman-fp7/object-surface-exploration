#include "icubFinger.h"

namespace objectExploration {

using std::cerr;
using std::endl;
using std::cout;

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



bool icubFinger::toArmPosition(Vector &fingertipPosition, Vector &retArmpPosition){


    bool ret = true;


  /*  Vector joints;
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
        joints[j] *= DEG2RAD;

    yarp::sig::Matrix tipFrame = _iCubFinger->getH(joints);
    Vector tip_x = tipFrame.getCol(3);*/

    Vector tip_x;
    getPositionHandFrameCorrected(tip_x);

    retArmpPosition.resize(3);
    retArmpPosition[0] = fingertipPosition[0] + tip_x[0];
    retArmpPosition[1] = fingertipPosition[1] + tip_x[1];
    retArmpPosition[2] = fingertipPosition[2] - tip_x[2];


    return ret;
}

/*bool icubFinger::getPosition(yarp::sig::Vector &position){
    bool ret;

    Vector fingerEncoders;
    fingerEncoders.resize(3);
    ret = readEncoders(fingerEncoders);

    ret = ret && getPosition(position, fingerEncoders);

    return ret;

}*/


/*bool icubFinger::getPosition(yarp::sig::Vector &position, yarp::sig::Vector &fingerEncoders){
    cerr << _dbgtag << "get position has not been implemented for this finger." << endl;
    return false;
}*/

/*bool icubFinger::getPositionHandFrame(yarp::sig::Vector &position){
    cerr << _dbgtag << "get position has not been implemented for this finger." << endl;


return false;

}*/

bool icubFinger::getPositionHandFrameCorrected(yarp::sig::Vector &position){
   return Finger::getPositionHandFrame(position);
}

bool icubFinger::getPositionHandFrameCorrected(yarp::sig::Vector &position, yarp::sig::Vector &fingerEncoders){
    return getPositionHandFrame(position, fingerEncoders);
}

bool icubFinger::getPositionHandFrame(yarp::sig::Vector &position, yarp::sig::Vector &fingerEncoders){
    cerr << _dbgtag << "get position has not been implemented for this finger." << endl;
    return false;
}


bool IndexFinger::getPositionHandFrameCorrected(yarp::sig::Vector &position){

    Finger::getPositionHandFrame(position);
    // Transfer it to the new frame

    yarp::sig::Matrix H0(4,4);
/*
R_hand =

    0.9928   -0.1104    0.0474
    0.1060    0.9906    0.0867
   -0.0565   -0.0810    0.9951


TT_hand =

   -0.0020
    0.0156
   -0.0035

   Free motion:
TR_hand =

    0.9928   -0.1005   -0.0657
    0.1060    0.9906    0.0867
    0.0564   -0.0930    0.9941


TT_hand =

   -0.0052
    0.0156
   -0.0072

   Freemotion 2

TR_hand =

    0.9934   -0.1093    0.0345
    0.1060    0.9906    0.0867
   -0.0437   -0.0824    0.9956


TT_hand =

   -0.0014
    0.0156
   -0.0033

*/

    // Should be able to choose based on left/right hand
    // Also should be able to disable it
   if(false){
    H0(0,0)=0.9934;   H0(0,1)=-0.1093; H0(0,2)=0.0345;  H0(0,3)=-0.0014;
    H0(1,0)=0.1060;   H0(1,1)=0.9906;  H0(1,2)=0.0867;  H0(1,3)=0;//0.0156;
    H0(2,0)=-0.0437;  H0(2,1)=-0.0824; H0(2,2)=0.9956;  H0(2,3)=-0.0033;
    H0(3,0)=0.0;      H0(3,1)=0.0;     H0(3,2)=0.0;     H0(3,3)=1.0;

   }
   else{
   H0(0,0)=1.0;   H0(0,1)=0.0; H0(0,2)=0.0;  H0(0,3)=0.0;
   H0(1,0)=0.0;   H0(1,1)=1.0; H0(1,2)=0.0;  H0(1,3)=0.0;//0.0156;
   H0(2,0)=0.0;   H0(2,1)=0.0; H0(2,2)=1.0;  H0(2,3)=0.0;
   H0(3,0)=0.0;   H0(3,1)=0.0; H0(3,2)=0.0;  H0(3,3)=1.0;
   }
    //T_rotoTrans = yarp::math::axis2dcm(armOrient);
    //T_rotoTrans.setSubcol(armPos, 0,3);

    //cout << position.toString() << endl;
    Vector retMat = yarp::math::operator *(H0, position);
    position = retMat.subVector(0,2);

}

bool IndexFinger::getPositionHandFrame(yarp::sig::Vector &position, yarp::sig::Vector &fingerEncoders){

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
    joints[0] = 20; // The index fingertip calibration procedure used this value.
    joints[1] = 90 * (1 - (fingerEncoders[0] - _minProximal) / (_maxProximal - _minProximal) );
    joints[2] = 90 * (1 - (fingerEncoders[1] - _minMiddle) / (_maxMiddle - _minMiddle) );
    joints[3] = 90 * (1 - (fingerEncoders[2] - _minDistal) / (_maxDistal - _minDistal) );

    //cout << joints.size() << endl;
    //Convert the joints to radians.
    for (int j = 0; j < joints.size(); j++)
        joints[j] *= DEG2RAD;


    yarp::sig::Matrix tipFrame = _iCubFinger->getH(joints);
    position = tipFrame.getCol(3); // Tip's position in the hand coordinate


    return ret;
}

/*bool IndexFinger::getPosition(yarp::sig::Vector &position, yarp::sig::Vector &fingerEncoders){


    // I am using an hybrid fingertip position forward kinematics. Fristly, I use the the actual encoder
    // data for the last three joints. Secondly, the behaviour of the icubFinger position estimation
    // is a little unpredictable, so I am transforming the fingertip position into the robot coordingates
    // myself.
    bool ret = true;
    Vector tip_x;

    getPositionHandFrame(tip_x, fingerEncoders);

    cout << "TipX: " << tip_x.toString() << endl;

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
}*/

icubFinger::icubFinger(t_controllerData ctrlData):Finger(ctrlData){
    _fingerEncoders = ctrlData.fingerEncoders;


}

void icubFinger::adjustMinMax(const double currentVal, double &min, double &max){


    if(currentVal > max)
        max = currentVal;
    if(currentVal < min)
        min = currentVal;


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

/*bool icubFinger::calibrate(){

    return true;
}*/

void IndexFinger::getTactileDataRaw(yarp::sig::Vector &rawTactileData){

    // Read from the port
    rawTactileData.resize(12);
    rawTactileData.zero();

    Bottle* tactileData;
    tactileData = _rawTactileData_in->read();

    //cout << tactileData->toString() << endl;
    int startIndex = 0;
    if(tactileData != NULL){
        for (int i = startIndex; i < 12; ++i)
        {
            rawTactileData[i - startIndex] = tactileData->get(i).asDouble();
        }
    }
}

void IndexFinger::getTactileDataComp(yarp::sig::Vector &tactileData){
    tactileData.resize(12);
    tactileData.zero();

    Bottle *tactileDataPort = _tactileDataComp_in->read();

    int startIndex = 0;
    if(tactileDataPort != NULL){
        for(int i = startIndex; i < startIndex + 12; ++i){
            tactileData[i - startIndex] = tactileDataPort->get(i).asDouble();
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
    _maxProximal = 253;
    _minProximal = 38;
    _maxMiddle = 251;
    _minMiddle = 61;
    _maxDistal = 255;
    _minDistal = 35;



}



bool IndexFinger::setSynchroProximalAngle(double angle){

    double distal = 90-angle;
    return setAngles(angle, distal, 40);
}

bool MiddleFinger::setSynchroProximalAngle(double angle){

    double distal = 90-angle;
    return setAngles(angle, distal, 40);
}



bool icubFinger::calibrate(){
    cerr << _dbgtag << "this finger has no calibration implemented" << endl;
    return true;
}

bool icubFinger::calibrateIndexMiddle(){

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

    // TODO: Why do I d this!
    checkMinMax(_minDistal, _maxDistal);
    checkMinMax(_minMiddle, _maxMiddle);
    checkMinMax(_minProximal, _maxProximal);

    cout << "Calibration mins:" <<
            _minProximal << "\t" <<
            _minMiddle << "\t" <<
            _minDistal << "\t" << endl;

    cout << "Calibration maxes:" <<
            _maxProximal << "\t" <<
            _maxMiddle << "\t" <<
            _maxDistal << "\t" << endl;
}


bool IndexFinger::calibrate(){
    return icubFinger::calibrateIndexMiddle();
}

bool IndexFinger::prepare(){

    bool ret = true;

    //setSynchroProximalAngle(0);
    ret = ret && setAngle(_distalJointIndex, 0);
    ret = ret && setAngle(_proximalJointIndex, 0);

    return ret;

}


bool MiddleFinger::calibrate(){
    return icubFinger::calibrateIndexMiddle();
}

bool MiddleFinger::prepare(){

    bool ret = true;

    //setSynchroProximalAngle(0);
    ret = ret && setAngle(_distalJointIndex, 0);
    ret = ret && setAngle(_proximalJointIndex, 0);

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
    _maxProximal = 228;
    _minProximal = 1;
    _maxMiddle = 243;
    _minMiddle = 28;
    _maxDistal = 254;
    _minDistal = 38;

}

bool MiddleFinger::getPositionHandFrame(yarp::sig::Vector &position, yarp::sig::Vector &fingerEncoders){
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
    position = tipFrame.getCol(3); // Tip's position in the hand coordinate

}

// Slightly different from the index finger. It is not affected by the abduction
bool MiddleFinger::getPosition(yarp::sig::Vector &position, yarp::sig::Vector &fingerEncoders){


    // I am using an hybrid fingertip position forward kinematics. Fristly, I use the the actual encoder
    // data for the last three joints. Secondly, the behaviour of the icubFinger position estimation
    // is a little unpredictable, so I am transforming the fingertip position into the robot coordingates
    // myself.

    bool ret = true;

    Vector tip_x;
    getPositionHandFrame(tip_x, fingerEncoders);

    cout << "TipX: " << tip_x.toString() << endl;
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


void MiddleFinger::getTactileDataRaw(yarp::sig::Vector &rawTactileData){

    // Read from the port
    rawTactileData.resize(12);
    rawTactileData.zero();

    Bottle* tactileData;
    tactileData = _rawTactileData_in->read();

    //cout << tactileData->toString() << endl;
    int startIndex = 12;
    if(tactileData != NULL){
        for (int i = startIndex; i < 12; ++i)
        {
            rawTactileData[i - startIndex] = tactileData->get(i).asDouble();
        }
    }
}

void MiddleFinger::getTactileDataComp(yarp::sig::Vector &tactileData){
    tactileData.resize(12);
    tactileData.zero();

    Bottle *tactileDataPort = _tactileDataComp_in->read();

    int startIndex = 12;
    if(tactileDataPort != NULL){
        for(int i = startIndex; i < startIndex + 12; ++i){
            tactileData[i - startIndex] = tactileDataPort->get(i).asDouble();
        }
    }

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
    // TODO: check if I can get them from the

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



}
