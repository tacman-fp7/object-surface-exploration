#pragma once
#include <explorationStrategyThread.h>
#include <gpExplorationThread.h>
#include <objectFeaturesThread.h>
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
#include "gpExplorationMultifinger.h"
#include "hand.h"
#include <fstream>

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
    
    //bool setHomePose();
    //bool goToHomePose();
    bool setStartingPose();
    bool goToStartingPose();
    bool setEndPose();
    bool goToEndPose();
    bool startExploringGrid(const std::string objectName);
    bool startExploring(const std::string& type, const std::string& objectName);

    bool stopExploring();
    bool fingerSetAngle(const double angle);
    bool openHand();
    bool prepHand();
    bool calibrateHand();
    bool startExploringGP(const string& objectName);
    bool startExploringMultifinger(const string& objectName);
    bool exploreGPSurface(const string& objectName);
    bool enableSurfaceSampling();
    bool disableSurfaceSampling();
    bool refineModelEnable();
    bool refineModelDisable();
    bool validatePositionsEnable();
    bool validatePositionsDisable();
    bool nRepeatsSet(const int32_t nRepeats);
    bool setHeight(double height);
    bool quit();
    bool alignFingers();
    
public: // Methods related to the RF module
    bool attach(yarp::os::Port &source);
    bool configure( yarp::os::ResourceFinder &rf );
    bool updateModule();
    bool close();

    
private:
    // members related to the rf module
    // The port for the robot control server
    yarp::os::Port _robotControl_port;





    
    // Different exploration strategies
    ExplorationStrategyThread *_exploreObjectThread; // run appropriate exploration strategy
    GPExplorationThread *_exploreObjectGP_thread; //TODO: fix this os there is only one pointer to the exploration strategy
    GPExplorationMultifingerThread *_exploreObjectMultifinger_thread;
    ExploreGPSurfaceThread * _exploreGPSurface_thread;



    yarp::os::ResourceFinder _rf;
    ObjectFeaturesThread* _objectFeaturesThread; // This is shared between threads. Must have sync

    int _explorationThreadPeriod;
    bool _exploreObjectOnOff;
    bool _exploreObjectValid;
    bool _stopModule;

    Hand* _robotHand;
    Finger* _explorationFinger;
    Finger* _auxiliaryFinger;

    std::string _dbgtag;
};
} // namespace objectExploration
