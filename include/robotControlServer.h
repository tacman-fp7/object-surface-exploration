#pragma once

#include <yarp/os/RFModule.h>
#include <robotControl.h>
#include <yarp/os/ResourceFinder.h>
#include <approachObject.h>
#include <string.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/sig/Vector.h>
#include <exploreObject.h>

using yarp::os::RFModule;

struct robotControlData
{

  std::string arm;
  std::string robotName;
  std::string controller;
  std::string controllerName;
};

typedef struct robotControlData t_robotControlData ;

class robotControlServer: public robotControl, public RFModule
{
public:
  robotControlServer();
  virtual ~robotControlServer();
  
  virtual bool setHomePose();
  virtual bool setStartingPose();
  virtual bool setEndPose();
  virtual bool goToHomePose();
  virtual bool goToStartingPose();
  virtual bool goToEndPose();
  
  
  //virtual bool goToHomePose();
  virtual bool updateHomePose();
  virtual bool updateContactPose();
  virtual bool approach();
  virtual bool contact();
  virtual bool explore(const bool onOff);
  virtual bool quit();
  //virtual bool setStartingPoint();
  //virtual bool setEndPoint();
  
  // RF module methods
  virtual bool attach(yarp::os::Port &source);
  virtual bool configure( yarp::os::ResourceFinder &rf );
  virtual bool updateModule();
  virtual bool close();
  

  
private:
  
  // The port for the robot control server
  yarp::os::Port _robotControl_port;
  
  // Flag to allow remote killing of the module
  bool _stopModule;
  
  // All functions are delegated to this
  objectExploration::ExploreObject* _exploreObject; 
  

  yarp::dev::PolyDriver _deviceController;
  
  t_robotControlData _robotcontrolData;
  

  
};

