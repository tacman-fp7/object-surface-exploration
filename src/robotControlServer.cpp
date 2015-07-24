
#include <robotControlServer.h>
#include <stdio.h>
#include <signal.h> 

robotControlServer::robotControlServer()
{
  _stopModule = false;
}


bool robotControlServer::approach()
{
  printf("Approaching the object.\n");
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
    return true;
}


bool robotControlServer::close()
{
  // Close neatly, this function is called when Ctl+C is registered
  _port.close();
  return true;
}


bool robotControlServer::updateModule()
{
  // Put a repetitive task here that will be run every getPeriod() time
  if(_stopModule)
    raise(SIGINT);
    //exit(EXIT_SUCCESS);
}

