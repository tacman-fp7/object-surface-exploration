#pragma once
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/ResourceFinder.h>
#include <string.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>
#include <yarp/os/Mutex.h>

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
  

namespace objectExploration
{

  
  class ObjectFeaturesThread: public RateThread
  {
  public:
    ObjectFeaturesThread(int period, ResourceFinder rf):RateThread(period), _tactileSum(0), _rf(rf){
      _armOrientation.resize(4);
      _armPosition.resize(3);
    };
    double getFingerForce(int nFinger){ return _tactileSum;};
    void run();
    bool threadInit();
    void threadRelease();
    Vector getPosition(){ 
      _armPoseMutex.lock();
      Vector temp = _armPosition;
      _armPoseMutex.unlock();
      return temp;}; 
    Vector getOrientation(){
      _armPoseMutex.lock();
      Vector temp = _armOrientation;
      _armPoseMutex.unlock();
      return temp;};
     double getForce()
     {
       _tactileMutex.lock();
       double temp = _tactileSum;
       _tactileMutex.unlock();
        
       return temp;
     }
  private:
    string _arm;
    string _robotName;
    string _controller;
    string _controllerName;
    
   
    BufferedPort<Bottle> _tactilePort;
    BufferedPort<Bottle> _armPositionPort; // TODO: This should be changed to the fingertip poistion
    ResourceFinder _rf;
    
    Mutex _tactileMutex;
    double _tactileSum;   
    
    Mutex _armPoseMutex;
    Vector _armPosition;
    Vector _armOrientation;
    
    // A container for the features
  };
  
} // End of namespace