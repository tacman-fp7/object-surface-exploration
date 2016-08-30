#pragma once
#include <string>
#include <yarp/sig/Vector.h>
#include <iCub/iKin/iKinFwd.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/os/BufferedPort.h>



namespace objectExploration {

enum fingerJoints{
    ABDUCTION = 7,
    THUMB_PROXIMAL = 9,
    THUMB_DISTAL = 10,
    INDEX_PROXIMAL = 11,
    INDEX_DISTAL = 12,
    MIDDLE_PROXIMAL = 13,
    MIDDLE_DISTAL = 14
};

enum fingerEncoders{
    // Double check the thumb data
    THUMB_PROXIMAL_ENCODER = 0, // TODO: check this
    THUMB_MIDDLE_ENCODER = 1,
    THUMB_DISTAL_ENCODER = 2,


    INDEX_PROXIMAL_ENCODER = 3,
    INDEX_MIDDLE_ENCODER = 4,
    INDEX_DISTAL_ENCODER = 5,

    // TODO: validate the values
    MIDDLE_PROXIMAL_ENCODER = 6,
    MIDDLE_MIDDLE_ENCODER = 7,
    MIDDLE_DISTAL_ENCODER = 8

};

struct fingerControllerData{
    std::string whichHand;
    std::string whichFinger;
    yarp::dev::IEncoders *armEncoder;
    yarp::dev::IControlMode2 *armJointModeCtrl;
    yarp::dev::IPositionControl *armJointPositionCtrl;
    yarp::dev::ICartesianControl *armCartesianCtrl;
    yarp::dev::IControlLimits *armLimits;
    yarp::os::BufferedPort<yarp::os::Bottle>* fingerEncoders;
    yarp::os::BufferedPort<yarp::os::Bottle>* rawTactileData_in;


};
typedef struct fingerControllerData t_controllerData;

using std::string;
using yarp::sig::Vector;
using yarp::os::Bottle;
using yarp::os::BufferedPort;
using yarp::os::ResourceFinder;


class Finger{


public:
    virtual bool prepare(){}
    bool open();
    virtual bool toArmPosition(Vector &fingertipPosition, Vector &retArmpPosition);
    bool setAngles(double proximal, double distal, double speed);
    bool setAngles(double proximal, double speed); //consider changing the name to something meaningful both distal and proximal are moves
    bool setProximalAngle(double angle, double speed = 30);
    bool setDistalAngle(double angle, double speed = 30);
    virtual bool setSynchroProximalAngle(double proximal){}
    virtual void calibrate();
    bool checkMotionDone();
    virtual bool getAngels(Vector &angles);
    virtual bool getPosition(yarp::sig::Vector &position);
    virtual bool getPosition(yarp::sig::Vector &position, yarp::sig::Vector &fingerEncoders);
    virtual bool readEncoders(Vector &encoderValues);
    double getContactForce();
    bool getContactCoP(yarp::sig::Vector& contactCoP);
    bool hasForceCoP();
    virtual void getRawTactileData(Vector& rawTactileData){rawTactileData.resize(12); rawTactileData.zero();}

protected:
    Finger(t_controllerData);
    bool setAngle(int joint, double angle, double speed = 30);
    double _prevContactForce;
    void alignJointsBounds();

private:
    static void initController(ResourceFinder& rf);


protected:
    yarp::dev::IControlMode2 *_armJointModeCtrl;
    yarp::dev::IPositionControl *_armJointPositionCtrl;
    yarp::dev::ICartesianControl *_armCartesianCtrl;
    yarp::dev::IControlLimits *_armControlLimits;

    string _dbgtag;
    yarp::dev::IEncoders* _armEncoder;

    iCub::iKin::iCubFinger* _iCubFinger;
    BufferedPort<Bottle> _contactForce_in;
    BufferedPort<Bottle> _contactCoP_in;

    BufferedPort<Bottle> _fingerControlPort_out;
    BufferedPort<Bottle>* _rawTactileData_in;

    int _proximalJointIndex;
    int _distalJointIndex;
    int _proximalEncoderIndex;
    int _middleEncoderIndex;
    int _distalEncoderIndex;

    double _curProximalAngle;
    double _curDistalAngle;

    string _whichFinger;
private:
    bool _isCoPValid;
    bool _isForceValid;
    bool _isActiveTaxelValid;


};

class simFinger:public Finger{
protected:
    simFinger(t_controllerData ctrlData);
};

class icubFinger:public Finger{
public:
    bool readEncoders(Vector &encoderValues);
    void calibrate();
    bool getAngels(Vector &angles);
    bool toArmPosition(Vector &fingertipPosition, Vector &retArmpPosition);
    bool getPosition(yarp::sig::Vector &position, yarp::sig::Vector &fingerEncoders);
    bool getPosition(yarp::sig::Vector &position);
protected:
    icubFinger(t_controllerData ctrlData);
    BufferedPort<Bottle>* _fingerEncoders;

protected:
    double _maxProximal;
    double _minProximal;
    double _maxMiddle;
    double _minMiddle;
    double _maxDistal;
    double _minDistal;



private:
    void adjustMinMax(const double currentVal, double &min, double &max);

protected:
    void getAngels(yarp::sig::Vector &angles, Vector fingerEncoders);

};

class SimIndexFinger: public Finger{
public:
    SimIndexFinger(t_controllerData ctrlData);
    void calibrate(){}
    bool prepare();
    bool setSynchroProximalAngle(double proximal);
    void getRawTactileData(Vector rawTactileData);
};

class SimThumb: public Finger{
public:
    SimThumb(t_controllerData ctrlData);
    bool prepare();
};

class IndexFinger: public icubFinger{

public:
    IndexFinger(t_controllerData);
    void calibrate();
    bool prepare();
    bool setSynchroProximalAngle(double proximal);
    void getRawTactileData(Vector& rawTactileData);
};



class MiddleFinger: public icubFinger{

public:
    MiddleFinger(t_controllerData ctrlData);
    bool prepare(){}
};

class Thumb:public icubFinger{

public:
    Thumb(t_controllerData ctrlData);
    bool prepare();

};


class FingerFactory{
public:
    Finger* createFinger(string whichFinger, string whichRobot, t_controllerData ctrlData
                         ){

        ctrlData.whichFinger = whichFinger;


        if(whichRobot.compare("icub") == 0){
            if(whichFinger.compare("index") == 0){
                return new IndexFinger(ctrlData);

            }
            if(whichFinger.compare("middle") == 0){
                return new MiddleFinger(ctrlData);
            }
            else if(whichFinger.compare("thumb") == 0){
                return new Thumb(ctrlData);
            }
        }
        else if(whichRobot.compare("icubSim") == 0){
            if(whichFinger.compare("index") == 0){
                return new SimIndexFinger(ctrlData);
            }
            if(whichFinger.compare("index") == 0){
                std::cerr << "NO Sim Middle finger defined" << std::endl;
                return NULL;
            }
            else if(whichFinger.compare("thumb") == 0){
                return new SimThumb(ctrlData);
            }
        }
    }
};

}//end of namespace
