#pragma once
#include <string>
#include <yarp/sig/Vector.h>
#include <iCub/iKin/iKinFwd.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/os/BufferedPort.h>
#include <stdexcept>
#include <deque>
#include <math.h>
#include <fstream>

namespace objectExploration {

#define DEG2RAD (M_PI/180)

enum fingerID{
    INDEX_FINGER_ID = 0,
    MIDDLE_FINGER_ID = 1
};

enum fingerJoints{
    ABDUCTION = 7,
    THUMB_PROXIMAL = 9,
    THUMB_DISTAL = 10,
    INDEX_PROXIMAL = 11,
    INDEX_DISTAL = 12,
    MIDDLE_PROXIMAL = 13,
    MIDDLE_DISTAL = 14,
    PINKY = 15
};

enum fingerEncoders{

    THUMB_PROXIMAL_ENCODER = 0,
    THUMB_MIDDLE_ENCODER = 1,
    THUMB_DISTAL_ENCODER = 2,


    INDEX_PROXIMAL_ENCODER = 3,
    INDEX_MIDDLE_ENCODER = 4,
    INDEX_DISTAL_ENCODER = 5,


    MIDDLE_PROXIMAL_ENCODER = 6,
    MIDDLE_MIDDLE_ENCODER = 7,
    MIDDLE_DISTAL_ENCODER = 8,

    RING_PROXIMAL_ENCODER = 9,
    RING_MIDDLE_ENCODER = 10,
    RING_DISTAL_ENCODER = 11,

    LITTLE_PROXIMAL_ENCODER = 12,
    LITTLE_MIDDLE_ENCODER = 13,
    LITTLE_DISTAL_ENCODER = 14,


};

struct fingerControllerData{
    std::string whichHand;
    std::string whichFinger;
    yarp::dev::IEncoders *armEncoder;
    yarp::dev::IControlMode2 *armJointModeCtrl;
    yarp::dev::IPositionControl *armJointPositionCtrl;
    yarp::dev::ICartesianControl *armCartesianCtrl;
    yarp::dev::IControlLimits *armControlLimits;
    yarp::os::BufferedPort<yarp::os::Bottle>* fingerEncoders;
    yarp::os::BufferedPort<yarp::os::Bottle>* rawTactileData_in;
    yarp::os::BufferedPort<yarp::os::Bottle> * tactileDataComp_in;
    yarp::os::ResourceFinder *rf; //to load parameters
};
typedef struct fingerControllerData t_controllerData;

using std::string;
using yarp::sig::Vector;
using yarp::os::Bottle;
using yarp::os::BufferedPort;
using yarp::os::ResourceFinder;

#define FORCE_TH 1
class Finger{


public:
    //Finger() it is a factory class. Do not implement this method
    ~Finger();
    bool open();
    virtual bool prepare();
    virtual bool calibrate();
    void logTactileCoP();

    virtual bool toArmPosition(Vector &fingertipPosition, Vector &retArmpPosition);


    virtual bool setAngles(double proximal, double distal, double speed);
    virtual bool setAngles(double proximal, double speed); //consider changing the name to something meaningful both distal and proximal are moves
    virtual bool setProximalAngle(double angle, double speed = 30);
    virtual bool setDistalAngle(double angle, double speed = 30);
    virtual bool setSynchroProximalAngle(double proximal){}
    int getFingerID(){
       if(_whichFinger.compare("index") == 0)
           return INDEX_FINGER_ID;
       else if(_whichFinger.compare( "middle") == 0)
           return MIDDLE_FINGER_ID;
    }

    bool checkMotionDone();
    virtual bool getAngels(Vector &angles);
    bool getPositionCoPAdjusted(Vector &position);
    bool getPosition(yarp::sig::Vector &position);
    bool getPositionCorrected(yarp::sig::Vector &position);

    bool getPosition(yarp::sig::Vector &position, yarp::sig::Vector &fingerEncoders);
    bool getPositionCorrected(yarp::sig::Vector &position, yarp::sig::Vector &fingerEncoders);

    bool getPositionHandFrame(Vector &position);

    virtual bool readEncoders(Vector &encoderValues);

    virtual bool getPositionHandFrameCorrected(yarp::sig::Vector &position);

    double getContactForce();

    /**
     * @brief getContactForceThreshold
     * @return the threshold for the contact force
     */
    double getContactForceThreshold();

    /**
     * @brief setContactForceThreshold
     */
    void setContactForceThreshold(const double threshold);

    bool getContactCoP(yarp::sig::Vector& contactCoP);
    bool hasForceCoP();
    virtual void getTactileDataRaw(Vector& rawTactileData){
        std::cerr << "Cannot get raw tactile data. Not implmented for this finger" << std::endl;
        rawTactileData.resize(12); rawTactileData.zero();}
    virtual void getTactileDataComp(Vector& tactileData){
        std::cerr << "Cannot get comp tactile data. Not implmented for this finger" << std::endl;
                     tactileData.resize(12); tactileData.zero();}
    //virtual bool calibrate2(){}
    virtual void printJointLimits(){}
    string getFingerName();




protected:
    virtual bool getPositionHandFrame(yarp::sig::Vector &position, yarp::sig::Vector &fingerEncoders);
    virtual bool getPositionHandFrameCorrected(yarp::sig::Vector &position, yarp::sig::Vector &fingerEncoders);

protected:
    Finger(t_controllerData);
    bool setAngle(int joint, double angle, double speed = 30);

    void alignJointsBounds();
    void checkMinMax(double &min, double &max);


private:
    static void initController(ResourceFinder& rf);
    void configure(std::string fingerName, yarp::os::ResourceFinder *rf);

protected:

    std::ofstream _tactileDataCompFile;
    std::ofstream _tactileDataRawFile;
    std::ofstream _copFile;

    double _prevContactForce;

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
    BufferedPort<Bottle>* _tactileDataComp_in;

    int _finerID;
    int _proximalJointIndex;
    int _distalJointIndex;

    int _proximalEncoderIndex;
    int _middleEncoderIndex;
    int _distalEncoderIndex;

    double _curProximalAngle;
    double _curDistalAngle;

    string _whichFinger;
    string _whichHand;

    double _contactForceThreshold; //Store finger specific force threshold that determins contact

    double _maxProximal;
    double _minProximal;
    double _maxMiddle;
    double _minMiddle;
    double _maxDistal;
    double _minDistal;

private:
    bool _isCoPValid;
    bool _isForceValid;
    bool _isActiveTaxelValid;



};








}//end of namespace
