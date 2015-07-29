
#include <robotControlServer.h>
#include <stdio.h>
#include <signal.h> 
#include <yarp/os/ResourceFinder.h>
#include <approachObjectManual.h>
#include <yarp/os/Bottle.h>
#include <yarp/math/Math.h>
#include <gsl/gsl_math.h>

using namespace yarp::os;
using namespace yarp::math;

robotControlServer::robotControlServer()
{
  _stopModule = false;
}


robotControlServer::~robotControlServer()
{
  //std::cout << "Destructor called." << std::endl; 
  _deviceController.close();
}

bool robotControlServer::updateContactPose()
{
  _exploreObject->updateContactPose();
  return true;
}

bool robotControlServer::updateHomePose()
{  
  _exploreObject->updateHomePose();
  return true;
}




bool robotControlServer::approach()
{
  printf("Approaching the object in the precontact position.\n");
  _exploreObject->approach();
  return true;
}

bool robotControlServer::goToHomePose()
{
  printf("Going to home position...");
  _exploreObject->goToHomePose();
  printf("done!\n");
  return true;
}

bool robotControlServer::contact()
{
  printf("Not implemented yet.\n");
  return true;
}

bool robotControlServer::explore()
{
  printf("Performing the exploratory behaviour.\n");
  _exploreObject->exploreObject(true); // implement toggle behaviour
  return true;
}

bool robotControlServer::quit()
{
 printf("Quitting\n");
 _stopModule = true;
 return true;
}


////////////////////////////////
// RF module related functions
////////////////////////////////

bool robotControlServer::attach(yarp::os::Port& source)
{
  return this->yarp().attachAsServer(source);
}





bool robotControlServer::configure(yarp::os::ResourceFinder& rf)
{
  
  bool ret = true;
  /////////
  std::string moduleName = rf.check("name",
            yarp::os::Value("robotControlServer"),
            "module name (string)").asString().c_str();
    setName(moduleName.c_str());
    
    attach(_port);
    
    std::string portName= "/";
    portName+= getName();
    if (!_port.open(portName.c_str())) {
        std::cout << getName() << ": Unable to open port " << portName << std::endl;
        return false;
    }
   
   
   
  // Get the configuration for the  arm
  yarp::os::Bottle &armConfig = rf.findGroup("Arm");
  if(!armConfig.isNull()){
   // Read the arm configuration
    _robotcontrolData.arm = armConfig.check("arm", Value("Error")).asString();
    _robotcontrolData.robotName = armConfig.check("robotName", Value("Error")).asString();
    _robotcontrolData.controller = armConfig.check("controller", Value("Error")).asString();
    _robotcontrolData.controllerName = armConfig.check("controllerName", Value("Error")).asString();
   
   std::cout << "Configuring the approachObject module:" << std::endl;
   std::cout << "Robot name: " << _robotcontrolData.robotName << std::endl;
   std::cout << "Arm: " << _robotcontrolData.arm << std::endl;
   std::cout << "Controller: " << _robotcontrolData.controller << std::endl;
   std::cout << "Controller name: " << _robotcontrolData.controllerName << std::endl;
  }
 

 /////////////////////////////// Configure the controller //////////////////////////////////
   yarp::os::Property deviceOptions;
   deviceOptions.put("device", _robotcontrolData.controller);
   deviceOptions.put("local", "/client_controller/" + _robotcontrolData.arm + "_arm");
   deviceOptions.put("remote", "/" + _robotcontrolData.robotName
   + "/" + _robotcontrolData.controllerName + "/" + _robotcontrolData.arm + "_arm");
   
   std::cout << deviceOptions.toString() << std::endl;
   
   if(!_deviceController.open(deviceOptions))
   {
      std::cout << "Failed to open the device: " << _robotcontrolData.controller << std::endl;
      ret = false;
    
   }
   
  _exploreObject = new objectExploration::ExploreObject(&_deviceController, rf);
  
   if(ret)
     std::cout << "Configuration completed." << std::endl;
   else
     std::cerr << "Failed to configure." << std::endl;
   return ret;
}


bool robotControlServer::close()
{
  // Close neatly, this function is called when Ctl+C is registered
  _port.close();
  _deviceController.close(); // Close the device controller
 
  
  return true;
}


bool robotControlServer::updateModule()
{
  // Put a repetitive task here that will be run every getPeriod() time
  if(_stopModule){
    printf("User requested to stop the module!\n");
    raise(SIGINT);
  }
    
}

