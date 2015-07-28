
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
  _approachObjectCntrl = new objectExploration::ApproachObjectManual;
}


robotControlServer::~robotControlServer()
{
  //std::cout << "Destructor called." << std::endl;
   if(_approachObjectCntrl != NULL)
    delete(_approachObjectCntrl);
   
  _deviceController.close();
}

bool robotControlServer::updateContactPose()
{
  Vector pos, orient;
  pos.resize(3); // x,y,z position 
  orient.resize(4); // x,y,z,w prientation
  _armCartesianController->getPose(pos, orient);
  _approachObjectCntrl->updateContactpose(pos, orient);
  
  return true;
}

bool robotControlServer::updateHomePose()
{
    Vector pos, orient;
  pos.resize(3); // x,y,z position 
  orient.resize(4); // x,y,z,w prientation
  _armCartesianController->getPose(pos, orient);
  _approachObjectCntrl->updateHomePose(pos, orient);
return true;
}




bool robotControlServer::approach()
{
  printf("Approaching the object.\n");
  // TODO: use mutex to make sure only one thread controls the robot.
  // At the moment I am running on a single thread.
  
  _approachObjectCntrl->approach(*_armCartesianController);
  return true;
}

bool robotControlServer::goToHomePose()
{
  _approachObjectCntrl->goToHomepose(*_armCartesianController);
  return true;
}

bool robotControlServer::contact()
{
  printf("Making contact.\n");
  return true;
}

bool robotControlServer::explore()
{
  printf("Performing the exploratory behaviour.\n");
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
   
   if(_deviceController.isValid())
       _deviceController.view(_armCartesianController);
  
   if(_armCartesianController == NULL)
   {
     std::cout << "Failed to open cartesian controller" << std::endl;
     ret = false;
   }
   
   
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
    
 /*t += 0.1;
  
  // translational target part: a circular trajectory
        // in the yz plane centered in [-0.3,-0.1,0.1] with radius=0.1 m
        // and frequency 0.1 Hz
        xd[0]=-0.3;
        xd[1]=-0.1+0.1*cos(2.0*M_PI*0.1*(t));
        xd[2]=+0.1+0.1*sin(2.0*M_PI*0.1*(t));

        // we keep the orientation of the left arm constant:
        // we want the middle finger to point forward (end-effector x-axis)
        // with the palm turned down (end-effector y-axis points leftward);
        // to achieve that it is enough to rotate the root frame of pi around z-axis
        od[0]=0.0; od[1]=0.0; od[2]=1.0; od[3]=M_PI;
	
	_armCartesianController->goToPose(xd, od);
	*/
}

