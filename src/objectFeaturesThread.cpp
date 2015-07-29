#include <objectFeaturesThread.h>
#include <yarp/os/Network.h>
#include <yarp/os/Value.h>
#include <yarp/os/Bottle.h>

using yarp::os::Network;
using yarp::os::Value;
using yarp::os::Bottle;
using std::cout;
using std::endl;
using std::cerr;

/*
 * 192 for hand data, where 1-60 are taxels of fingertips (12 each in this order:
 * index, middle, ring, little, thumb); 61-96 zeros; 97-144 palm taxels 
 * (inside these, 108, 120, 132, and 140 are thermal pads ~ 0s); 145-192 zeros.
 */
void objectExploration::ObjectFeaturesThread::run()
{
  Bottle* tactileData = _tactilePort.read(true); // Wait for data
  
  if(tactileData->isNull()){
    cerr << "Did not receive tactile data" << endl;
    return;
  }
  cout << tactileData->size() << endl;
  
 
  // The first 12 are for the index finger, I am only using the 4 on the tip
  _tactileSum = tactileData->get(1).asDouble();
  _tactileSum = tactileData->get(2).asDouble();
  _tactileSum = tactileData->get(10).asDouble();
  _tactileSum = tactileData->get(11).asDouble();
  
  cout << _tactileSum << endl;
}


bool objectExploration::ObjectFeaturesThread::threadInit()
{
  yarp::os::RateThread::threadInit();
  
  bool ret = true;
  // This is where I need to connect the port
  // TODO: This one definitely needs finger data, like whic finger
 
  Bottle &armConfig = _rf.findGroup("Arm");
  if(!armConfig.isNull()){
   // Read the arm configuration
    _arm = armConfig.check("arm", Value("left")).asString();
    _robotName = armConfig.check("robotName", Value("icub")).asString();
   
   
   cout << "Configuring the objectFeaturesThread:" << endl;
   cout << "Robot name: " << _robotName << endl;
   cout << "Arm: " << _arm << endl << endl;
   
  }
  
  
  if(!_tactilePort.open("/objectExploration/tactileSensors/" + _arm + "_hand")){
   ret = false;
   printf("Failed to open tactile port\n");
  }
  
  yarp::os::Network::connect("/" + _robotName + "/skin/" + _arm + "_hand_comp",
    "/objectExploration/tactileSensors/" + _arm + "_hand");
  
  return ret;
}

void objectExploration::ObjectFeaturesThread::threadRelease()
{
yarp::os::RateThread::threadRelease();

_tactilePort.close();
}

