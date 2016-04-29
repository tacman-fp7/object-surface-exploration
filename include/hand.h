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

namespace objectExploration {

struct reachableSpace
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
    bool prepare();
    bool open();
    bool setAbduction(double angle, double speed = 20);
    bool getPose(Vector& pos, Vector& orient);
    bool calibrate();
    void setEndPose(Vector& pos, Vector& orient);
    void setStartingPose(Vector& pos, Vector& orient);
    bool getStartingPose(Vector& pos, Vector& orient);
    bool getEndPose(Vector& pos, Vector& orient);
    bool moveToPosition(Vector pos, Vector orient);
    bool goToStartingPose();
    bool goToEndPose();
    bool checkOpenMotionDone();

private:
    void configure(ResourceFinder rf);
    void updateRobotReachableSpace();
    void printPose(Vector& pos, Vector& orient);

private:
    string _whichHand;
    Finger* _indexFinger; // Think of a better way that will allow multiple fingers
    Finger* _thumb;

    yarp::dev::IEncoders *_armEncoders;
    yarp::dev::IControlMode2 *_armJointModeCtrl;
    yarp::dev::IPositionControl *_armJointPositionCtrl;
    //yarp::dev::IPositionControl* _armJointPositionController;

    yarp::dev::PolyDriver _deviceController; // The use depends on the view
    yarp::dev::ICartesianControl* _armCartesianCtrl;

    yarp::dev::PolyDriver _deviceController_joint;

    bool _desiredStartingPose_isValid;
    Vector _desiredStartingPosition;
    Vector _desiredStartingOrientation;
    //double _zMax;
    //double _zMin;

    bool _desiredEndPose_isValid;
    Vector _desiredEndPosition;
    Vector _desiredEndOrientation;

    bool _homePose_isValid;
    Vector _homePosition;
    Vector _homeOrientation;

    reachableSpace _robotReachableSpace;
    //////
    ///
private:

    // yarp::dev::IControlMode2* _armController_mode;
    int _cartCtrlStartupIDstartupID; // Context ID of the controller at the start
    string _dbgtag;
};
}
