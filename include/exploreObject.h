#pragma once
#include <approachObject.h>
#include <maintainContactThread.h>
#include <explorationStrategyThread.h>
#include <objectFeaturesThread.h>
#include <objectClassifierThread.h>
#include <yarp/dev/CartesianControl.h>
#include <approachObject.h>
#include <approachObjectManual.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/ResourceFinder.h>

// Explore object interface for various object exploration strategies

namespace objectExploration
{
  class ExploreObject
  {
  public:
    ExploreObject(yarp::dev::PolyDriver* deviceController, yarp::os::ResourceFinder& rf);
    ~ExploreObject();
    bool approach();
    bool goToHomePose();
    bool updateHomePose();
    bool updateContactPose();
    bool setEndPose();
    bool goToEndPose();
  
    //bool approachObject(){/*do nothing at the moment.*/ };
    bool maintainContact(bool onOff){return false;/*do nothing at the moment.*/ };
    bool exploreObject(bool onOff);
    
  private:
    // It has to be instantiated with the desired approachObject instance
    ApproachObject* _approachObjectCntrl; // Approach the object
    MaintainContactThread* _maintainContactThread; // maintain contact
    ExplorationStrategyThread* _exploreObjectThread; // run appropriate exploration strategy
    ObjectClassifierThread* _objectClassifierThread; // run appropriate classifier
    yarp::os::ResourceFinder _rf;
    
  private:
    ObjectFeaturesThread* _objectFeaturesThread; // This is shared between threads. Must have sync
    yarp::dev::PolyDriver* _deviceController; // The view depends on the use
    yarp::dev::ICartesianControl* _armCartesianController;
    bool _exploreObjectOnOff;
  };
} // End of namespace