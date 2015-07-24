/*
 * Author: Nawid Jamali
 * Project: TACMAN
 */
#include <stdio.h>
#include <yarp/os/Time.h>
#include <iostream>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Image.h>
#include <robotControlServer.h>

using namespace std;
using namespace yarp::os;


int main(int argc, char *argv[])
{	
  Network yarp;
  if (!yarp.checkNetwork())
  {
    printf("Could not contact yarp server, quitting.\n");
    return false;
  }
  
  yarp::os::ResourceFinder rf;
  rf.setVerbose(true);
  rf.configure(argc, argv);
    
  // Create a robot control server
  robotControlServer robotControllerModule;
  
  if(!robotControllerModule.configure(rf))
    return 1;
  
  return robotControllerModule.runModule();
  
  return 0;
}
