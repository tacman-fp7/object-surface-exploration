#include "simFinger.h"

namespace objectExploration {


SimIndexFinger::SimIndexFinger(t_controllerData ctrlData):
    Finger(ctrlData){


    _iCubFinger = new iCub::iKin::iCubFinger(ctrlData.whichHand + "_index");
    //alignJointsBounds();
    // Joint indexes as defined in iCub
    _proximalJointIndex = INDEX_PROXIMAL;
    _distalJointIndex = INDEX_DISTAL;

    _proximalEncoderIndex = INDEX_PROXIMAL_ENCODER;
    _middleEncoderIndex = INDEX_MIDDLE_ENCODER;
    _distalEncoderIndex = INDEX_DISTAL_ENCODER;


}

SimMiddleFinger::SimMiddleFinger(t_controllerData ctrlData): Finger(ctrlData){
    _iCubFinger = new iCub::iKin::iCubFinger(ctrlData.whichHand + "_middle");

    // Joint indexes as defined in iCub
    _proximalJointIndex = MIDDLE_PROXIMAL;
    _distalJointIndex = MIDDLE_DISTAL;

    _proximalEncoderIndex = MIDDLE_PROXIMAL_ENCODER;
    _middleEncoderIndex = MIDDLE_MIDDLE_ENCODER;
    _distalEncoderIndex = MIDDLE_DISTAL_ENCODER;
}

bool SimIndexFinger::setSynchroProximalAngle(double angle){

    double distal = 90-angle;
    return setAngles(angle, distal, 40);
}

bool SimMiddleFinger::setSynchroProximalAngle(double angle){
    double distal = 90-angle;
    return setAngles(angle, distal, 40);
}

bool SimIndexFinger::prepare(){

    bool ret = true;

    _curProximalAngle = 0;
    _curDistalAngle = 90;

    ret = ret && setAngle(_proximalJointIndex, _curProximalAngle);
    ret = ret && setAngle(_distalJointIndex, _curDistalAngle);

    return ret;

}

bool SimMiddleFinger::prepare(){

    bool ret = true;

    _curProximalAngle = 0;
    _curDistalAngle = 90;

    ret = ret && setAngle(_proximalJointIndex, _curProximalAngle);
    ret = ret && setAngle(_distalJointIndex, _curDistalAngle);

    return ret;

}


SimThumb::SimThumb(t_controllerData ctrlData):
    Finger(ctrlData){

    //Finger::Finger(armEncoder, armJointModeCtrl, armJointPositionCtrl);




    _iCubFinger = new iCub::iKin::iCubFinger(ctrlData.whichHand + "_thumb");
    //alignJointsBounds();
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
