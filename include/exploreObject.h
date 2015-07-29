#pragma once
#include <approachObject.h>
#include <maintainContactThread.h>
#include <explorationStrategyThread.h>
#include <objectFeatures.h>
#include <objectClassifierThread.h>
#include <yarp/dev/CartesianControl.h>
#include <approachObject.h>
#include <approachObjectManual.h>
#include <yarp/dev/PolyDriver.h>

// Explore object interface for various object exploration strategies

namespace objectExploration
{
  class ExploreObject
  {
  public:
    ExploreObject(yarp::dev::PolyDriver* deviceController);
    ~ExploreObject();
    bool approach();
    bool goToHomePose();
    bool updateHomePose();
    bool updateContactPose();
  
    //bool approachObject(){/*do nothing at the moment.*/ };
    bool maintainContact(bool toggle){/*do nothing at the moment.*/ };
    bool exploreObject(bool toggle){/*do nothing at the moment.*/ };
    
  private:
    // It has to be instantiated with the desired approachObject instance
    ApproachObject* _approachObjectCntrl; // Approach the object
    MaintainContactThread* _maintainContactThread; // maintain contact
    ExplorationStrategyThread* _exploreObjectThread; // run appropriate exploration strategy
    ObjectClassifierThread* _objectClassifierThread; // run appropriate classifier
    
  private:
    ObjectFeatures _objectFeatures; // This is shared between threads. Must have sync
    yarp::dev::PolyDriver* _deviceController; // The view depends on the use
    yarp::dev::ICartesianControl* _armCartesianController;
  };
} // End of namespace