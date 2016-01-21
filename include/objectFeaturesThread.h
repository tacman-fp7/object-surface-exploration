#pragma once
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/ResourceFinder.h>
#include <string.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/RpcClient.h>
#include <string>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IControlMode2.h>

// This object is used to update features which will be shared between object objectExploraton
// and object classification threads
// I can also add suggested trajectory which can be used by maintain contact module to 
// change trajectory/location

// Make it a thread that reads the finger data, sums 

using yarp::os::RateThread;
using yarp::os::BufferedPort;
using yarp::os::Bottle;
using yarp::os::ResourceFinder;
using yarp::sig::Vector;
using yarp::os::Mutex;
using std::string;
using yarp::os::RpcClient;

namespace objectExploration
{


class ObjectFeaturesThread: public RateThread
{
private:
    void adjustMinMax(const double currentVal, double &min, double &max);

public:
    ObjectFeaturesThread(int period, ResourceFinder rf);
    ~ObjectFeaturesThread();
    //double getFingerForce(int nFinger){ return _contactForce;}
    void run();
    bool threadInit();
    void threadRelease();
    //////// accessros and mutators ////
    Vector getPosition();
    Vector getOrientation();
    double getContactForce();
    bool getArmPose(Vector& pos, Vector& orient);
    void setEndPose(Vector& pos, Vector& orient);
    void setStartingPose(Vector& pos, Vector& orient);
    bool getDesiredEndPose(Vector& pos, Vector& orient);
    bool getStartingPose(Vector& pos, Vector& orient);
    void setHomePose(Vector& pos, Vector& orient);
    bool getHomePose(Vector& pos, Vector& orient);
    void setWayPoint(Vector pos, Vector orient);
    bool getWayPoint(Vector& pos, Vector& orient, bool invalidateWayPoint = true);
    bool readParameters();
    const string& getArm();
    const string& getRobotName();
    const string& getControllerType();
    const string& getControllerName();
    const int& getTrajectoryTime();
    const int& getMaintainContactPeriod();
    const int& getExplorationThreadPeriod();
    const double& getDesiredForce();
    void writeToFingerController(std::string command);
    void setArmController_jnt(yarp::dev::IEncoders *encoder, yarp::dev::IPositionControl *jointCtrl);
    void setArmController_cart(yarp::dev::ICartesianControl * cartesianCtrl);
    void setArmController_mode(yarp::dev::IControlMode2 *armJointCtrlmode);
    bool isExplorationValid(){return _isExplorationValid;}
    double getProximalJointAngle(){return _proximalJointAngle;}
    bool openHand();
    bool prepHand();
    void adjustIndexFinger();
    bool getFingertipZ(double *zDisp);
    bool getFingertipZ(double *zDisp, double proximalAngle);
    bool getFingertipZ(double *zDisp, Vector &fingerEncoders);
    void openIndexFinger();
    void calibrateHand();
    void getIndexFingerEncoder(Vector &encoderValues);
    bool fingerMovePosition(int joint, double angle, double speed = 10);

    bool checkOpenHandDone()
    {
        bool ret;
        if(!_armJointPositionCtrl->checkMotionDone(&ret))
        {
            std::cerr << _dbgtag << "CheckMotionDone failed on network comms" << std::endl;
            ret = true;
        }
                //ret = false;

        return ret;
    }

    bool setProximalAngle(double angle){
        if(_armJointPositionCtrl != NULL || _armEncoder !=NULL)
        {

           /* double encVal;
            _armEncoder->getEncoder(_proximalJoint_index,&encVal);

            std::cout << "Encoder: "  << encVal << std::endl;
            */

            //double aa;
            //getFingertipZ(&aa);
            // Make sure the control is in position mode
            //_armJointPositionCtrl->setPositionMode();
            _armJointModeCtrl->setPositionMode(11);
            return (_armJointPositionCtrl->positionMove(_proximalJoint_index, angle));

        }
        std::cerr << _dbgtag << "The joint controller is not initialised" << std::endl;
        return false;
    }
    bool getFingertipPose(Vector& pos, Vector& orient);

private:
    void printPose(Vector& pos, Vector& prient);
protected:
    ResourceFinder _rf;

    /////// Robot parameters ///////////
    string _arm;
    string _robotName;
    string _controller;
    string _controllerName;
    string _moduleName;
    string _whichFinger;

    int _trajectoryTime;


    //////// Finger encoders //////

    BufferedPort<Bottle> _fingerEncoders;
    double _maxIndexProximal;
    double _minIndexProximal;
    double _maxIndexMiddle;
    double _minIndexMiddle;
    double _maxIndexDistal;
    double _minIndexDistal;

    ////// Exploration parameters ///////
    int _maintainContactPeriod;
    int _readTactilePeriod;
    int _explorationThreadPeriod;
    bool _isExplorationValid;

    double _desiredFroce;
    Mutex _desiredForceMutex;


    bool _desiredStartingPose_isValid;
    Vector _desiredStartingPosition;
    Vector _desiredStartingOrientation;
    double _zMax;
    double _zMin;

    bool _desiredEndPose_isValid;
    Vector _desiredEndPosition;
    Vector _desiredEndOrientation;

    bool _homePose_isValid;
    Vector _homePosition;
    Vector _homeOrientation;

    //////// Object Features ////////////
    //BufferedPort<Bottle> _tactilePort;
   // BufferedPort<Bottle> _armPositionPort;
    Mutex _tactileMutex;
    //double _tactileSum;
    BufferedPort<Bottle> _contactForceCoPPort;
    double _contactForce;


    /// Using Massimo's controller to maintainContact
    //RpcClient _fingerController_RPC;
    BufferedPort<Bottle> _fingerController_port;
    string _fingerControllerPortName;

    /// Clean them a little later /////

    ///
    Mutex _armJointMutex;
    Vector _armJoints;

    /// Arm pose and orient is actually gripper pose and orient!!!!
    Mutex _armPoseMutex;
    Vector _armPosition;
    Vector _armOrientation;

    bool _wayPoint_isValid;
    Vector _wayPointPos;
    Vector _wayPointOrient;

    std::string _dbgtag;// = "objectFeaturesThread.cpp: ";

    //// The port to read the joint information

    yarp::dev::IEncoders *_armEncoder;
    yarp::dev::ICartesianControl *_armCartesianCtrl;
    yarp::dev::IPositionControl *_armJointPositionCtrl;
    yarp::dev::IControlMode2 *_armJointModeCtrl;

    double _proximalJointAngle;
    int _proximalJoint_index;

    //BufferedPort<Bottle> _armJointPort_in;

};

} // namespace objectExploration
