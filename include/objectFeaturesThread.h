#pragma once
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/ResourceFinder.h>
#include <string.h>

// This object is used to update features which will be shared between object objectExploraton
// and object classification threads
// I can also add suggested trajectory which can be used by maintain contact module to 
// change trajectory/location

// Make it a thread that reads the finger data, sums 

  using yarp::os::RateThread;
  using yarp::os::BufferedPort;
  using yarp::os::Bottle;
  using yarp::os::ResourceFinder;
  using std::string;
  

namespace objectExploration
{

  
  class ObjectFeaturesThread: public RateThread
  {
  public:
    ObjectFeaturesThread(int period, ResourceFinder rf):RateThread(period), _tactileSum(0), _rf(rf){};
    double getFingerForce(int nFinger){ return _tactileSum;};
    void run();
    bool threadInit();
    void threadRelease();
  private:
    double _tactileSum;   
    string _arm;
    string _robotName;
    string _controller;
    string _controllerName;
    
   
    BufferedPort<Bottle> _tactilePort;
    BufferedPort<Bottle> _armPositionPort; // TODO: This should be changed to the fingertip poistion
    ResourceFinder _rf;
    
    // A container for the features
  };
  
} // End of namespace