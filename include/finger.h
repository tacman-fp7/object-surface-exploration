#pragma once
#include <string>
#include <yarp/sig/Vector.h>
#include <iCub/iKin/iKinFwd.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/CartesianControl.h>

namespace objectExploration {

enum fingerJoints{
    ABDUCTION = 7,
    THUMB_PROXIMAL = 9,
    THUMB_DISTAL = 10,
    INDEX_PROXIMAL = 11,
    INDEX_DISTAL = 12
};

enum fingerEncoders{
    // Double check the thumb data
    THUMB_PROXIMAL_ENCODER = 1,
    THUMB_MIDDLE_ENCODER = 2,
    THUMB_DISTAL_ENCODER = 3,


    INDEX_PROXIMAL_ENCODER = 3,
    INDEX_MIDDLE_ENCODER = 4,
    INDEX_DISTAL_ENCODER = 5

};

struct fingerControllerData{
    std::string whichHand;
    yarp::dev::IEncoders *armEncoder;
    yarp::dev::IControlMode2 *armJointModeCtrl;
    yarp::dev::IPositionControl *armJointPositionCtrl;
    yarp::dev::ICartesianControl* armCartesianCtrl;

};
typedef struct fingerControllerData t_controllerData;

using std::string;
using yarp::sig::Vector;
using yarp::os::Bottle;
using yarp::os::BufferedPort;
using yarp::os::ResourceFinder;


class Finger{


public:
    virtual bool prepare() =0;
    bool open();
    bool toArmPosition(Vector &fingertipPosition, Vector &retArmpPosition);
    bool setAngles(double proximal, double distal, double speed);
    bool setAngles(double proximal, double speed); //consider changing the name to something meaningful both distal and proximal are moves
    bool setProximalAngle(double angle, double speed = 30);
    bool setDistalAngle(double angle, double speed = 30);
    bool setSynchroProximalAngle(double proximal);
    void calibrate();
    bool checkMotionDone();
    bool getAngels(Vector &angles);
    bool getPosition(yarp::sig::Vector &position);
    bool getPosition(yarp::sig::Vector &position, yarp::sig::Vector &fingerEncoders);
    bool readEncoders(Vector &encoderValues);
protected:
    Finger(t_controllerData);
    bool setAngle(int joint, double angle, double speed = 30);
    //bool getEncoderValues(Vector &encoderValues);
    void getAngels(yarp::sig::Vector &angles, Vector fingerEncoders);


private:
    void adjustMinMax(const double currentVal, double &min, double &max);

    static void initController(ResourceFinder& rf);

private:


    yarp::dev::IEncoders* _armEncoder;
    yarp::dev::IControlMode2 *_armJointModeCtrl;
    yarp::dev::IPositionControl *_armJointPositionCtrl;

    yarp::dev::ICartesianControl* _armCartesianCtrl;


    string _dbgtag;

protected:
    double _maxProximal;
    double _minProximal;
    double _maxMiddle;
    double _minMiddle;
    double _maxDistal;
    double _minDistal;


BufferedPort<Bottle> _fingerEncoders;
    iCub::iKin::iCubFinger* _iCubFinger;

    int _proximalJointIndex;
    int _distalJointIndex;


    int _proximalEncoderIndex;
    int _middleEncoderIndex;
    int _distalEncoderIndex;



};

class IndexFinger: public Finger{

public:
    IndexFinger(t_controllerData);
    void calibrate();
    bool prepare();
    bool setSynchroProximalAngle(double proximal);
};



class Thumb:public Finger{

public:
    Thumb(t_controllerData ctrlData);
    bool prepare();

};


class FingerFactory{
public:
    Finger* createFinger(string whichFinger, t_controllerData ctrlData
                         ){

        if(whichFinger.compare("index") == 0){
            return new IndexFinger(ctrlData);
        }
        else if(whichFinger.compare("thumb") == 0){
            return new Thumb(ctrlData);
        }
    }
};

}//end of namespace
