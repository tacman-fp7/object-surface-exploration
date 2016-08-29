#pragma once
#include <vector>
//#include <yarp/dev/IEncoders.h>
#include "finger.h"
#include <string>
#include <finger.h> // This should be Finger Factory only I guess
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>

//#include "contactSafetyThread.h"

namespace objectExploration {

struct workspace
{
    double minX;
    double maxX;
    double minY;
    double maxY;
    double minZ;
    double maxZ;
    double disasterX;
};

using yarp::os::ResourceFinder;
class Hand{
public:
    Hand( ResourceFinder rf);
   ~Hand();
    bool prepare();
    bool open();
    bool setAbduction(double angle, double speed = 20);
    bool getPose(Vector& pos, Vector& orient);
    bool calibrate();
    bool goToPoseSync(yarp::sig::Vector& pos, yarp::sig::Vector& orient, double timeout = 0.0);
    void stopControl();
    bool checkOpenMotionDone(); //Rename to reduce confusion
    bool checkMotionDone(bool* motionDone);
    void waitMotionDone(double period, double timeout = 0.0);

    void setEndPose(Vector& pos, Vector& orient);
    void setStartingPose(Vector& pos, Vector& orient);
    bool getStartingPose(Vector& pos, Vector& orient);
    bool getEndPose(Vector& pos, Vector& orient);
    bool setWayPoint(Vector pos);
    bool setWayPoint(Vector pos, Vector orient);
    bool setWayPointGP(Vector pos, Vector orient);
    bool getWayPoint(Vector& pos, Vector& orient, bool invalidateWayPoint = true);
    bool goToStartingPose();
    bool goToEndPose();
    bool setHeight(double height);
    bool multiContact(double angle);
    //void relaxTolerence();
    //void strictTolerence();


    Finger* getIndexFinger(){return _indexFinger;}
    Finger* getMiddleFinger(){return _middleFinger;}
    string getArmName(){return _whichHand;}
    string getRobotName(){return _robotName;}



protected:
    void configure(ResourceFinder rf);

private:

    void updateSafeWorkspace();
    void printPose(Vector& pos, Vector& orient);

protected:
    string _moduleName;
    string _whichHand;
    string _robotName;

    Finger* _indexFinger; // Think of a better way that will allow multiple fingers
    Finger* _middleFinger;
    Finger* _thumb;



    yarp::dev::IEncoders *_armEncoders;
    yarp::dev::IControlMode2 *_armJointModeCtrl;
    yarp::dev::IPositionControl *_armJointPositionCtrl;
    yarp::dev::ICartesianControl* _armCartesianCtrl;
    //ContactSafetyThread* _contactSafetyThread;

private:

    yarp::dev::PolyDriver _deviceController_joint;
    yarp::dev::PolyDriver _deviceController; // The use depends on the view
    bool _desiredStartingPose_isValid;
    Vector _desiredStartingPosition;
    Vector _desiredStartingOrientation;


    bool _desiredEndPose_isValid;
    Vector _desiredEndPosition;
    Vector _desiredEndOrientation;

    bool _homePose_isValid;
    Vector _homePosition;
    Vector _homeOrientation;

    workspace _safeWorkspace;

    bool _wayPoint_isValid;
    Vector _wayPointPos;
    Vector _wayPointOrient;

   BufferedPort<Bottle> _abuductionPort_out;
protected:
   BufferedPort<Bottle> _rawTactileData_in;

private:


    int _cartCtrlStartupIDstartupID; // Context ID of the controller at the start
    string _dbgtag;
};

class SimHand: public Hand{

public:
    SimHand(ResourceFinder& rf);
    bool configure(ResourceFinder& rf);
};

class icubHand: public Hand{
public:
    icubHand(ResourceFinder& rf);
    bool configure(yarp::os::ResourceFinder rf);

protected:
    BufferedPort<Bottle> _fingerEncoders;

};
}
