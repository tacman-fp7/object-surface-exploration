#include "finger.h"

#include <iostream>
#include <math.h>

namespace objectExploration {

using std::cerr;
using std::endl;



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
    Vector fingerEncoders;

    ret = ret && readEncoders(fingerEncoders);
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
    Vector tip_x = tipFrame.getCol(3);

    retArmpPosition.resize(3);
    retArmpPosition[0] = fingertipPosition[0] + tip_x[0];
    retArmpPosition[1] = fingertipPosition[1] + tip_x[1];
    retArmpPosition[2] = fingertipPosition[2] - tip_x[2];


    return ret;
}

bool Finger::prepare(){

}

Finger::Finger(t_controllerData ctrlData){
    _armEncoder = ctrlData.armEncoder;
    _armJointModeCtrl = ctrlData.armJointModeCtrl;
    _armJointPositionCtrl = ctrlData.armJointPositionCtrl;
}

bool Finger::readEncoders(Vector &encoderValues)
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
        encoderValues[0] = handEnc->get(_proximalEncoderIndex).asDouble();
        encoderValues[1] = handEnc->get(_middleEncoderIndex).asDouble();
        encoderValues[2] = handEnc->get(_distalEncoderIndex).asDouble();
        ret = true;
    }

    //cout << "Encoders: " << encoderValues.toString() << endl;
    return ret;
}

bool Finger::setAngles(double proximalAngle, double speed){



    double distalAngle = 90 - proximalAngle;
    return setAngles(proximalAngle, distalAngle, speed);



}

void Finger::adjustMinMax(const double currentVal, double &min, double &max){


    if(currentVal > max)
        max = currentVal;
    if(currentVal < min)
        min = currentVal;


}

bool Finger::setAngle(int jointIndex, double angle, double speed){

    bool ret = true;

    ret = ret && _armJointModeCtrl->setPositionMode(jointIndex);;
    ret = ret && _armJointPositionCtrl->setRefSpeed(jointIndex, speed);
    ret = ret && _armJointPositionCtrl->positionMove(jointIndex, speed);

    return ret;
}

bool Finger::setAngles(double proximal, double distal, double speed)
{
    bool ret = true;

    ret = ret && setAngle(_proximalJointIndex, proximal, speed);
    ret = ret && setAngle(_distalJointIndex, distal, speed);

    return ret;
}


IndexFinger::IndexFinger(t_controllerData ctrlData):
    Finger(ctrlData){


    _iCubFinger = new iCub::iKin::iCubFinger(ctrlData.whichHand + "_index");

    // Joint indexes as defined in iCub
    _proximalJointIndex = INDEX_PROXIMAL;
    _distalJointIndex = INDEX_DISTAL;

    _proximalEncoderIndex = INDEX_PROXIMAL_ENCODER;
    _middleEncoderIndex = INDEX_MIDDLE_ENCODER;
    _distalEncoderIndex = INDEX_DISTAL_ENCODER;



}

bool IndexFinger::prepare(){

    bool ret = true;

    ret = ret && Finger::setAngle(_distalJointIndex, 20);
    ret = ret && Finger::setAngle(_proximalJointIndex, 20);

    return ret;

}

Thumb::Thumb(t_controllerData ctrlData):
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

bool Thumb::prepare(){
    bool ret = true;

    ret = ret && Finger::setAngle(_proximalJointIndex, 0);
    ret = ret && Finger::setAngle(_distalJointIndex, 65);

    return ret;
}
}
