#pragma once
#include "finger.h"

namespace objectExploration{
class icubFinger:public Finger{
public:
    bool readEncoders(Vector &encoderValues);
    virtual bool calibrate();

    bool getAngels(Vector &angles);
    bool toArmPosition(Vector &fingertipPosition, Vector &retArmpPosition);

    //virtual bool getPositionHandFrame(yarp::sig::Vector &position);

    //virtual bool getPosition(yarp::sig::Vector &position, yarp::sig::Vector &fingerEncoders);
    //virtual bool getPosition(yarp::sig::Vector &position);
    void printJointLimits();
protected:
    virtual bool getPositionHandFrame(yarp::sig::Vector &position, yarp::sig::Vector &fingerEncoders);
    virtual bool getPositionHandFrameCorrected(yarp::sig::Vector &position, yarp::sig::Vector &fingerEncoders);
    icubFinger(t_controllerData ctrlData);
    BufferedPort<Bottle>* _fingerEncoders;
    bool calibrateIndexMiddle();
public:

    virtual bool getPositionHandFrameCorrected(yarp::sig::Vector &position);




private:
    void adjustMinMax(const double currentVal, double &min, double &max);


protected:
    void getAngels(yarp::sig::Vector &angles, Vector fingerEncoders);

};



class IndexFinger: public icubFinger{

public:
    IndexFinger(t_controllerData);
    bool calibrate();
    bool prepare();
    //bool getPositionHandFrame(yarp::sig::Vector &position);
    bool getPositionHandFrame(yarp::sig::Vector &position, yarp::sig::Vector &fingerEncoders);
    //virtual bool getPositionHandFrame(yarp::sig::Vector &position);
    //bool getPosition(yarp::sig::Vector &position, yarp::sig::Vector &fingerEncoders);
    virtual bool setSynchroProximalAngle(double proximal);
    void getTactileDataRaw(Vector& rawTactileData);
    void getTactileDataComp(Vector& tactileData);

public:
    bool getPositionHandFrameCorrected(yarp::sig::Vector &position);

};




class MiddleFinger: public icubFinger{

public:
    MiddleFinger(t_controllerData ctrlData);
    bool getPositionHandFrame(yarp::sig::Vector &position, yarp::sig::Vector &fingerEncoders);
    bool getPosition(yarp::sig::Vector &position, yarp::sig::Vector &fingerEncoders);
    virtual bool setSynchroProximalAngle(double proximal);
    bool prepare();
    bool calibrate();
    void getTactileDataRaw(Vector& rawTactileData);
    void getTactileDataComp(Vector& tactileData);
};

class RingAndLittleFingers: public icubFinger{
public:
    RingAndLittleFingers(t_controllerData ctrlData);

public:
    virtual bool setAngles(double proximal, double distal, double speed);
    virtual bool setAngles(double proximal, double speed); //consider changing the name to something meaningful both distal and proximal are moves
    virtual bool setProximalAngle(double angle, double speed = 30);
    virtual bool setDistalAngle(double angle, double speed = 30);
    virtual bool setSynchroProximalAngle(double proximal);

};

class RingFinger: public RingAndLittleFingers{
public:
    RingFinger(t_controllerData ctrlData);

};

class LittleFinger: public RingAndLittleFingers{
public:
    LittleFinger(t_controllerData ctrlData);
};

class Thumb:public icubFinger{

public:
    Thumb(t_controllerData ctrlData);
    bool prepare();

};


}// End of namespace
