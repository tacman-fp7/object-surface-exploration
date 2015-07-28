#pragma once

#include <yarp/os/RFModule.h>
#include <robotControl.h>
#include <yarp/os/ResourceFinder.h>
#include <approachObject.h>
#include <string.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/sig/Vector.h>


struct robotControlData
{
  //std::string device;
  //std::string local;
  //std::string remote;
  //std::string remoteTactile;
  std::string arm;
  std::string robotName;
  std::string controller;
  std::string controllerName;
};

typedef struct robotControlData t_robotControlData ;

class robotControlServer: public robotControl, public yarp::os::RFModule
{
public:
  robotControlServer();
  virtual ~robotControlServer();
  
  virtual bool goToHomePose();
  virtual bool updateHomePose();
  virtual bool updateContactPose();
  virtual bool approach();
  virtual bool contact();
  virtual bool explore();
  virtual bool quit();
  
  // RF module methods
  virtual bool attach(yarp::os::Port &source);
  virtual bool configure( yarp::os::ResourceFinder &rf );
  virtual bool updateModule();
  virtual bool close();
  

  
private:
  yarp::os::Port _port;
  bool _stopModule;
  objectExploration::ApproachObject* _approachObjectCntrl; 
  t_robotControlData _robotcontrolData;
  yarp::dev::PolyDriver _deviceController;
  yarp::dev::ICartesianControl* _armCartesianController;
    
  yarp::sig::Vector xd, od; // Testing only
  double t; // Testing only
  
  
  
};

