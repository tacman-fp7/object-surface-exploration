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
      _desiredEndPose_isValid = false;
      _desiredStartingPose_isValid = false;
      _wayPoint_isValid = false;
      _wayPointOrient.resize(4);
      _wayPointPos.resize(3);
      _desiredEndOrientation.resize(4);
      _desiredEndPosition.resize(3);
      _desiredStartingOrientation.resize(4);
      _desiredStartingPosition.resize(3);
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
     
     void setEndPose(Vector& pos, Vector& orient)
     {
	_desiredEndPosition = pos;
	_desiredEndOrientation = orient;
	_desiredEndPose_isValid = true;
     }
     
     void setStartingPose(Vector& pos, Vector& orient)
     {
       _desiredStartingPosition = pos;
       _desiredStartingOrientation = orient;
       _desiredStartingPose_isValid = true;
       
     }
     
     bool getDesiredEndPose(Vector& pos, Vector& orient)
     {
       if(_desiredEndPose_isValid)
       {
	 pos = _desiredEndPosition;
	 orient = _desiredEndOrientation;
       }
       return _desiredEndPose_isValid;
    };
    
    void setWayPoint(Vector pos, Vector orient)
    {
	_wayPointPos = pos;
	_wayPointOrient = orient;
	_wayPoint_isValid = true;
    }
    
    bool getWayPoint(Vector& pos, Vector& orient, bool invalidateWayPoint = true)
    {
	  if(_wayPoint_isValid)
	  {
	      pos = _wayPointPos;
	      orient = _wayPointOrient;
	      _wayPoint_isValid = !invalidateWayPoint;
	      return true;
	  }
	  return false;
    };
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
    
  protected: // Make them private later 
    bool _desiredStartingPose_isValid;
    Vector _desiredStartingPosition;
    Vector _desiredStartingOrientation;
    
    bool _desiredEndPose_isValid;
    Vector _desiredEndPosition;
    Vector _desiredEndOrientation;
    
    bool _wayPoint_isValid;
    Vector _wayPointPos;
    Vector _wayPointOrient;
    
    // A container for the features
  };
  
} // End of namespace