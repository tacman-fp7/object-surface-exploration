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
using yarp::os::Mutex;
/*
 * 192 for hand data, where 1-60 are taxels of fingertips (12 each in this order:
 * index, middle, ring, little, thumb); 61-96 zeros; 97-144 palm taxels 
 * (inside these, 108, 120, 132, and 140 are thermal pads ~ 0s); 145-192 zeros.
 */
void objectExploration::ObjectFeaturesThread::run()
{
 
  ///// Read the tactile data ///////
  Bottle* tactileData = _tactilePort.read(true); // Wait for data
  
  //// Read the corresponding arm position. //////
  ///  TODO: this should be changed to the fingertip position ///
  Bottle* armPose = _armPositionPort.read(true);
  if(tactileData->isNull() || armPose->isNull()){
    cerr << "Did not receive tactile or arm data" << endl;
    return;
  }
  
  
  _armPoseMutex.lock();
  for (int i = 0; i < 3; i++)
    _armPosition[i] = armPose->get(i).asDouble();
  for (int i = 3; i < 7; i++)
    _armOrientation[i] = armPose->get(i).asDouble();
  _armPoseMutex.unlock();
  
  //cout << armPose->toString() << endl << endl;
  //cout << _armPosition.toString() << endl;
  //cout << _armOrientation.toString() << endl << endl;
  
  _tactileMutex.lock();
  // The first 12 are for the index finger, I am only using the 4 on the tip
  _tactileSum = tactileData->get(12).asDouble();
  _tactileSum += tactileData->get(13).asDouble();
  _tactileSum += tactileData->get(21).asDouble();
  _tactileSum += tactileData->get(22).asDouble();
  _tactileSum  /= 4;
  _tactileMutex.unlock();
  
 /* cout << "Buffer size:" << tactileData->size() << endl;
  cout << "Tactile data:" << endl;
  for (int i = 12; i < 24; i++)
    cout << tactileData->get(i).toString() << " ";
  cout << endl;
  cout << "Tactile average: " << _tactileSum << endl;
  */
}


bool objectExploration::ObjectFeaturesThread::threadInit()
{
  yarp::os::RateThread::threadInit();
  
  bool ret = true;
  // This is where I need to connect the port
  // TODO: This one definitely needs finger data, like whic finger
 
  Bottle &armConfig = _rf.findGroup("Arm");
  Bottle* startingPose;
  if(!armConfig.isNull()){
   // Read the arm configuration
    _arm = armConfig.check("arm", Value("left")).asString();
    _robotName = armConfig.check("robotName", Value("icubSim")).asString();
    _controller = armConfig.check("controller", Value("Error")).asString();
    _controllerName = armConfig.check("controllerName", Value("Error")).asString(); 
    startingPose = armConfig.find("startingPose").asList();
  
  }
  
  //////// Initialise the startingPose //////
  if(startingPose->size() < 7)
    cout << "startingPose is invalid" << endl;
  else
  {
   for(int i = 0; i < 3; i++)
     _desiredStartingPosition[i] = startingPose->get(i).asDouble();
   for(int i = 3; i < 7; i++)
     _desiredStartingOrientation[i-3] = startingPose->get(i).asDouble();
   _desiredStartingPose_isValid = true;
  }
  
   cout << "Configuring the objectFeaturesThread:" << endl;
   cout << "Robot name: " << _robotName << endl;
   cout << "Arm: " << _arm << endl;
   cout << "Controller: " << _controller << endl;
   cout << "Controller name: " << _controllerName << endl;
   if(_desiredStartingPose_isValid)
   {
    cout << "Starting position: " << _desiredStartingPosition.toString() << endl;
    cout << "Starting orientation: " << _desiredStartingOrientation.toString() << endl;
   }
   cout << endl;
   
   //-0.249457	-0.138679	 0.160771 :
   //0.073738	-0.049514	 0.996048	 2.517341
   _desiredEndPosition.resize(3);
   _desiredEndPosition[0] = -0.249457;
   _desiredEndPosition[1] = -0.138679;
   _desiredEndPosition[2] = 0.160771;
   
   _desiredEndOrientation.resize(4);
   _desiredEndOrientation[0] = 0.073738;
   _desiredEndOrientation[1] = -0.049514;
   _desiredEndOrientation[2] = 0.996048;
   _desiredEndOrientation[3] = 2.517341;
   
   _desiredEndPose_isValid = true;
  
  
  
  /////////////////// Connect to the tactile sensor port /////////////////
  if(!_tactilePort.open("/objectExploration/tactileSensors/" + _arm + "_hand")){
   ret = false;
   printf("Failed to open local tactile port\n");
  }
  
  Network::connect("/" + _robotName + "/skin/" + _arm + "_hand_comp",
    "/objectExploration/tactileSensors/" + _arm + "_hand");
  
  
  /////////////// Opening amr pose port and connecting to it //////////////
  if(!_armPositionPort.open("/objectExploration/" + _arm + "_arm/pose"))
  {
   ret = false;
   cout << "Failed to open local arm pose port" << endl;
  }
  //icubSim/cartesianController/left_arm/state:o
  if(!Network::connect("/" + _robotName + "/" + _controllerName + "/" + _arm + "_arm/state:o",
    "/objectExploration/" + _arm + "_arm/pose"))
  {
   ret = false;
   cerr << "Failed to connect to the arm pose port" << endl;
  }
  
  // TODO: figure out why removing this crashes the application
  // is it because the the network connection needs time?
  if(ret)
    cout << "Object feagtures thread configured" << endl;
  else
    cerr << "Error, object features thread failed during configuration" << endl;
  
  return ret;
}

void objectExploration::ObjectFeaturesThread::threadRelease()
{
yarp::os::RateThread::threadRelease();


}

objectExploration::ObjectFeaturesThread::~ObjectFeaturesThread()
{
  
  _tactilePort.close();
  _armPositionPort.close();
}
