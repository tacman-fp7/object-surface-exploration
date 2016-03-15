#pragma once
//#include <approachObject.h>
//#include <maintainContactThread.h>
#include <explorationStrategyThread.h>
#include <gpExplorationThread.h>
#include <objectFeaturesThread.h>
//#include <objectClassifierThread.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IControlMode2.h>
#include <yarp/os/RFModule.h>
#include <robotControl.h>
#include <surfaceModelGP.h>
#include <exploreGPSurfaceThread.h>

using yarp::os::RFModule;

// Explore object interface for various object exploration strategies

namespace objectExploration
{
class ExploreObject: public robotControl, public RFModule
{
public:
    ExploreObject(yarp::os::ResourceFinder& rf);
    ~ExploreObject();


public: // Methods related to the robot control
    
    bool setHomePose();
    bool goToHomePose();
    bool setStartingPose();
    bool goToStartingPose();
    bool setEndPose();
    bool goToEndPose();
    bool startExploring();
    bool stopExploring();
    bool fingerSetAngle(const double angle);
    bool openHand();
    bool prepHand();
    bool calibrateHand();
    bool startExploringGP();
    bool exploreGPSurface(const string& objectName);
    bool quit();
    
public: // Methods related to the RF module
    bool attach(yarp::os::Port &source);
    bool configure( yarp::os::ResourceFinder &rf );
    bool updateModule();
    bool close();

    
private: // members related to the rf module
    // The port for the robot control server
    yarp::os::Port _robotControl_port;

    
private: // Private members
    // It has to be instantiated with the desired approachObject instance
    //ApproachObject* _approachObjectCntrl; // Approach the object
//    MaintainContactThread* _maintainContactThread; // maintain contact
    ExplorationStrategyThread *_exploreObjectThread; // run appropriate exploration strategy
    GPExplorationThread *_exploreObjectGP_thread;
    ExploreGPSurfaceThread * _exploreGPSurface_thread;

//    ObjectClassifierThread* _objectClassifierThread; // run appropriate classifier
    yarp::os::ResourceFinder _rf;
    ObjectFeaturesThread* _objectFeaturesThread; // This is shared between threads. Must have sync
    yarp::dev::PolyDriver _deviceController; // The view depends on the use
    yarp::dev::ICartesianControl* _armCartesianController;
    yarp::dev::PolyDriver _deviceController_joint;
    yarp::dev::IEncoders* _armEncoders;
    yarp::dev::IControlMode2* _armController_mode;

    yarp::dev::IPositionControl *_armJointPositionController;

    int _cartCtrlStartupIDstartupID; // Context ID of the controller at the start

    bool _exploreObjectOnOff;
    bool _exploreObjectValid;
    bool _stopModule;

    std::string _dbgtag;
};
} // namespace objectExploration
