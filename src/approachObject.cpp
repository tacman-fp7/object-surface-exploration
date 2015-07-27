#include <approachObject.h>
#include <string.h> 
#include <iostream>

using namespace yarp::os;

objectExploration::ApproachObject::ApproachObject()
{
  _initContactPos.resize(3);
}

/*bool objectExploration::ApproachObject::configure(ResourceFinder& rf)
{
  yarp::os::Bottle &armConfig = rf.findGroup("Arm");
  if(!armConfig.isNull()){
   // Read the arm configuration
   std::string device = armConfig.check("device", Value("Error")).asString();
   std::string local = armConfig.check("local", Value("Error")).asString();
   std::string remote = armConfig.check("remote", Value("Error")).asString();
   
   std::cout << "Configuring the approachObject module:" << std::endl;
   std::cout << "Device: " << device << std::endl;
   std::cout << "Local: " << local << std::endl;
   std::cout << "Remote: " << remote << std::endl;
   
  }
}

*/