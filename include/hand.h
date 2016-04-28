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


using yarp::os::ResourceFinder;
class Hand{
public:
    Hand( ResourceFinder rf);
    bool prepare();
    bool setAbduction(double angle, double speed = 20);

private:
    void configure(ResourceFinder rf);

private:
    string _whichHand;
    Finger* _indexFinger; // Think of a better way that will allow multiple fingers
    Finger* _thumb;

    yarp::dev::IEncoders *_armEncoders;
    yarp::dev::IControlMode2 *_armJointModeCtrl;
    yarp::dev::IPositionControl *_armJointPositionCtrl;


    //////
    ///
private:
    yarp::dev::PolyDriver _deviceController; // The use depends on the view
    yarp::dev::ICartesianControl* _armCartesianController;
    yarp::dev::IPositionControl* _armJointPositionController;
    yarp::dev::PolyDriver _deviceController_joint;
    // yarp::dev::IControlMode2* _armController_mode;
    int _cartCtrlStartupIDstartupID; // Context ID of the controller at the start
    string _dbgtag;
};
}
