#include <exploreObject.h>
#include <approachObjectManual.h>
#include <yarp/sig/Vector.h>
#include <signal.h>
#include <yarp/os/Bottle.h>
#include <planarExplorationThread.h>

using std::cout;
using std::endl;

objectExploration::ExploreObject::ExploreObject(yarp::dev::PolyDriver* deviceController,
						yarp::os::ResourceFinder& rf)
{

  bool failed = false;
  _exploreObjectOnOff = true;
  _rf = rf;
  _deviceController = deviceController;
  
  if(_deviceController->isValid())
  {
    //TODO: Check for successful creation of the object
    if(!_deviceController->view(_armCartesianController))
      printf("Failed to get a cartesian view\n");
    else
      failed = true;
  }
  else{
   printf("The device driver is not valid. Aborting!\n");
   failed = true;
  }
  _approachObjectCntrl = new ApproachObjectManual; 
  
  ///////////// Use the the resourcr finder to configure ///////////
  int readTactilePeriod;
  int maintainContactPeriod;
  int explorationThreadPeriod;
  
  double desiredForce;
  yarp::os::Bottle& exploreObjectConfig = rf.findGroup("ExploreObject");
  if(!exploreObjectConfig.isNull()){
    maintainContactPeriod = exploreObjectConfig.check("maintainContactPeriod", 5).asInt();
    desiredForce = exploreObjectConfig.check("desiredForce", 0.0).asDouble();
    readTactilePeriod = exploreObjectConfig.check("readTactilePeriod", 10).asInt();
    explorationThreadPeriod = exploreObjectConfig.check("explorationThreadPeriod", 10).asInt();
    
    printf("\n");
    printf("Explore object config data:\n");
    printf("Maintain-contact period: %d\n", maintainContactPeriod);
    printf("desiredForce: %0.2f\n", desiredForce);
    printf("Read-tactile period: %d\n", readTactilePeriod);
    printf("exploration thread period period: %d\n", explorationThreadPeriod);
    printf("\n");
  }
  else
  {
   printf("Failed to locate ExploreObject configuration\n");
   failed = true;
  }
  

  
  ////////// Setting up the tactile data reading thread ////////////
  _objectFeaturesThread = new ObjectFeaturesThread(readTactilePeriod, rf);
  _objectFeaturesThread->start();
  
    ////////// Setting up the MaintainContactThread ///////////////////////
  _maintainContactThread = new MaintainContactThread(maintainContactPeriod, _objectFeaturesThread);
  _maintainContactThread->setDesiredForce(desiredForce);
  
  _exploreObjectThread = new PlanarExplorationThread(explorationThreadPeriod, 
    _armCartesianController,_objectFeaturesThread);
  
  
  //if(failed)
  //  raise(SIGINT);
}

objectExploration::ExploreObject::~ExploreObject()
{
  if (_approachObjectCntrl != NULL)
    delete(_approachObjectCntrl);
  
  if(_maintainContactThread != NULL)
  {
    _maintainContactThread->stop();
    delete(_maintainContactThread);
  }
  
  if(_objectFeaturesThread != NULL)
  {    
    _objectFeaturesThread->stop();
    delete(_objectFeaturesThread);
  }
  
  if(_exploreObjectThread !=NULL)
  {
     _objectFeaturesThread->stop();
     delete(_objectFeaturesThread);
  }
  
}

bool objectExploration::ExploreObject::approach()
{
  
  return _approachObjectCntrl->approach(*_armCartesianController);
}


bool objectExploration::ExploreObject::goToHomePose()
{
 return _approachObjectCntrl->goToHomepose(*_armCartesianController);
}

bool objectExploration::ExploreObject::goToEndPose()
{
  return _approachObjectCntrl->goToEndPose(*_armCartesianController);
}

bool objectExploration::ExploreObject::updateContactPose()
{
  Vector pos, orient;
  pos.resize(3); // x,y,z position 
  orient.resize(4); // x,y,z,w prientation
  _armCartesianController->getPose(pos, orient);
  _approachObjectCntrl->updateContactpose(pos, orient);
  
  /// TODO: One of them is redundant
  
 _objectFeaturesThread->setStartingPose(pos, orient);
  //_objectFeaturesThread->
  return true;
}

bool objectExploration::ExploreObject::setEndPose()
{
  Vector pos, orient;
  pos.resize(3); // x,y,z position 
  orient.resize(4); // x,y,z,w prientation
  _armCartesianController->getPose(pos, orient);
  _approachObjectCntrl->setEndPose(pos, orient);
  
  /// TODO: One of them is redundant
  _objectFeaturesThread->setEndPose(pos, orient);
  return true;
}

bool objectExploration::ExploreObject::updateHomePose()
{
   Vector pos, orient;
  pos.resize(3); // x,y,z position 
  orient.resize(4); // x,y,z,w prientation
  _armCartesianController->getPose(pos, orient);
  _approachObjectCntrl->updateHomePose(pos, orient);
return true;
}

bool objectExploration::ExploreObject::exploreObject(bool onOff)
{
  bool ret = true;
  
  
  if(_exploreObjectOnOff)
  {
   //TODO: do some checks if the thread is running on so on
  // First step is to reach the pre-contact location
  if(!_approachObjectCntrl->approach(*_armCartesianController))
    ret = false;
  // Then explore the object
  if(!_maintainContactThread->start())
    ret = false;
  
  if(!_exploreObjectThread->start())
    ret = false;
  
  cout << "Exoploring the object\n" << endl;
  _exploreObjectOnOff = false;
  
  }
  else{
    
    if(!_approachObjectCntrl->goToHomepose(*_armCartesianController))
      ret = false;
    
    _maintainContactThread->stop();
     
    _exploreObjectThread->stop();
    
    cout << "Stopped the exploration" << endl;
    _exploreObjectOnOff = true;
  }
  
  return ret;
}

















